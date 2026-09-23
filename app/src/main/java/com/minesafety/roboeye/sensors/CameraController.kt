package com.minesafety.roboeye.sensors

import android.Manifest
import android.content.Context
import android.content.pm.PackageManager
import android.graphics.ImageFormat
import android.graphics.SurfaceTexture
import android.media.ImageReader
import android.util.Size
import android.view.Surface
import androidx.camera.core.CameraSelector
import androidx.camera.core.ImageAnalysis
import androidx.camera.core.Preview
import androidx.camera.core.SurfaceRequest
import androidx.camera.core.resolutionselector.AspectRatioStrategy
import androidx.camera.core.resolutionselector.ResolutionSelector
import androidx.camera.core.resolutionselector.ResolutionStrategy
import androidx.camera.lifecycle.ProcessCameraProvider
import androidx.core.content.ContextCompat
import androidx.lifecycle.LifecycleOwner
import com.minesafety.roboeye.core.Availability
import com.minesafety.roboeye.core.CameraResolution
import com.minesafety.roboeye.core.CameraRunState
import com.minesafety.roboeye.core.CameraState
import com.minesafety.roboeye.core.Logbook
import com.minesafety.roboeye.net.FramePayload
import com.minesafety.roboeye.perception.PerceptionEngine
import com.minesafety.roboeye.perception.PerceptionFrameResult
import java.util.concurrent.ExecutorService
import java.util.concurrent.Executors
import java.util.concurrent.atomic.AtomicBoolean
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.suspendCancellableCoroutine
import kotlin.coroutines.resume

/**
 * Owns the unified CameraX pipeline for Robo Eye:
 * - Live video streaming via [FramePayload] and [YuvJpeg]
 * - On-device atmospheric visibility scoring via [VisibilityEstimator]
 * - Device-aware, decoupled on-device AI perception via [PerceptionEngine]
 */
class CameraController(private val context: Context, private val logbook: Logbook) {

  private val _state = MutableStateFlow(initialState())
  val state: StateFlow<CameraState> = _state.asStateFlow()

  /**
   * Invoked for every sampled frame with visibility result.
   */
  var onFrame: ((jpeg: ByteArray?, result: VisibilityResult) -> Unit)? = null

  /**
   * Invoked for live video streaming with sequence and metadata.
   */
  var onLiveFrame: ((payload: FramePayload, result: VisibilityResult) -> Unit)? = null

  /**
   * Invoked for Rover Brain decoupled frame processing with direct Y-buffer.
   */
  var onCameraFrame: ((frame: com.minesafety.roboeye.core.model.CameraFrame) -> Unit)? = null

  /**
   * Dedicated bounded mailbox and worker context for neural perception.
   */
  private val perceptionMailbox = com.minesafety.roboeye.perception.PerceptionMailbox()

  /**
   * Invoked when on-device perception produces detected objects.
   */
  var onPerceptionResult: ((result: PerceptionFrameResult) -> Unit)?
    get() = perceptionMailbox.onResult
    set(value) { perceptionMailbox.onResult = value }

  /**
   * Optional neural perception engine (YOLO / SSD MobileNet).
   */
  var perceptionEngine: PerceptionEngine?
    get() = perceptionMailbox.engine
    set(value) { perceptionMailbox.engine = value }

  var phonePerceptionEnabled: Boolean
    get() = perceptionMailbox.isEnabled
    set(value) {
      if (value) {
        perceptionMailbox.onEnabled()
      } else {
        perceptionMailbox.onDisabled()
      }
    }

  var sensitivity: Float
    get() = perceptionMailbox.sensitivity
    set(value) { perceptionMailbox.sensitivity = value }

  @Volatile var maxPerceptionHz: Float = 4.0f

  var onPerceptionFrameSkipped: (() -> Unit)?
    get() = perceptionMailbox.onFrameSkipped
    set(value) { perceptionMailbox.onFrameSkipped = value }

  init {
    perceptionMailbox.onError = { e ->
      logbook.warn(TAG, "Phone perception pass failed: ${e.message}")
    }
  }

  private val executor: ExecutorService = Executors.newSingleThreadExecutor()

  private var provider: ProcessCameraProvider? = null
  private var analysis: ImageAnalysis? = null
  private var previewUseCase: Preview? = null
  private var surfaceProvider: Preview.SurfaceProvider? = null
  private var owner: LifecycleOwner? = null

  @Volatile private var resolution: CameraResolution = CameraResolution.HD720
  @Volatile private var intervalMs: Long = 83L // ~12 FPS default
  @Volatile private var jpegQuality: Int = 70
  @Volatile private var frameSeq: Long = 0L
  @Volatile private var lastAnalysisAt = 0L
  @Volatile private var lastVisibilityAt = 0L
  @Volatile private var lastPerceptionAt = 0L

  private val yBufPool = Array<java.nio.ByteBuffer?>(3) { null }
  private var yBufIndex = 0
  @Volatile private var cachedVisibility: VisibilityResult? = null
  @Volatile private var analysed = 0L
  @Volatile private var uploaded = 0L
  @Volatile private var dropped = 0L

  private val fpsWindow = ArrayDeque<Long>()

  private fun initialState(): CameraState {
    val hasCamera =
      context.packageManager.hasSystemFeature(PackageManager.FEATURE_CAMERA_ANY)
    return CameraState(
      availability =
        when {
          !hasCamera -> Availability.UNAVAILABLE
          !hasPermission() -> Availability.NO_PERMISSION
          else -> Availability.WAITING
        },
      detail = if (!hasCamera) "No camera on this device" else "",
    )
  }

  fun hasPermission(): Boolean =
    ContextCompat.checkSelfPermission(context, Manifest.permission.CAMERA) ==
      PackageManager.PERMISSION_GRANTED

  fun refreshPermission() {
    if (_state.value.runState == CameraRunState.STOPPED) _state.value = initialState()
  }

  suspend fun start(
    lifecycleOwner: LifecycleOwner,
    cameraResolution: CameraResolution,
    frameIntervalMs: Long,
    quality: Int = 70,
  ) {
    if (!hasPermission()) {
      _state.value =
        _state.value.copy(
          availability = Availability.NO_PERMISSION,
          runState = CameraRunState.STOPPED,
          detail = "Camera permission not granted",
        )
      return
    }
    owner = lifecycleOwner
    resolution = cameraResolution
    intervalMs = frameIntervalMs
    jpegQuality = quality.coerceIn(30, 95)
    _state.value = _state.value.copy(runState = CameraRunState.STARTING, detail = "")

    val cameraProvider =
      provider
        ?: runCatching { awaitProvider() }
          .onFailure { e ->
            fail("Camera provider unavailable: ${e.message ?: e::class.java.simpleName}")
          }
          .getOrNull()
        ?: return
    provider = cameraProvider
    bind()
  }

  private val headlessSurfaceProvider = object : Preview.SurfaceProvider {
    private var imageReader: ImageReader? = null
    private var surfaceTexture: SurfaceTexture? = null
    private var surface: Surface? = null

    override fun onSurfaceRequested(request: SurfaceRequest) {
      val width = request.resolution.width
      val height = request.resolution.height
      logbook.info(TAG, "Headless preview surface requested: ${width}x${height}")

      val surf: Surface = try {
        val reader = ImageReader.newInstance(width, height, ImageFormat.PRIVATE, 2).apply {
          setOnImageAvailableListener({ r ->
            try {
              val img = r.acquireLatestImage()
              img?.close()
            } catch (_: Throwable) {}
          }, null)
        }
        imageReader?.close()
        imageReader = reader
        reader.surface
      } catch (e: Exception) {
        logbook.warn(TAG, "ImageReader allocation failed (${e.message}), falling back to SurfaceTexture")
        val st = SurfaceTexture(0).apply {
          setDefaultBufferSize(width, height)
          detachFromGLContext()
        }
        surfaceTexture?.release()
        surfaceTexture = st
        Surface(st)
      }

      surface?.release()
      surface = surf

      request.provideSurface(surf, executor) { result ->
        logbook.info(TAG, "Headless surface result: ${result.resultCode}")
        surf.release()
        imageReader?.close()
        imageReader = null
        surfaceTexture?.release()
        surfaceTexture = null
        if (surface === surf) surface = null
      }
    }

    fun release() {
      surface?.release()
      surface = null
      imageReader?.close()
      imageReader = null
      surfaceTexture?.release()
      surfaceTexture = null
    }
  }

  fun setPreviewSurface(sp: Preview.SurfaceProvider?) {
    surfaceProvider = sp
    val targetProvider = sp ?: headlessSurfaceProvider
    logbook.info(TAG, "Preview surface updated: ${if (sp != null) "UI PreviewView" else "Headless fallback"}")
    previewUseCase?.setSurfaceProvider(executor, targetProvider)
  }

  private fun bind() {
    val cameraProvider = provider ?: return
    val lifecycleOwner = owner ?: return

    val aspectStrategy =
      if (resolution == CameraResolution.HD720) {
        AspectRatioStrategy.RATIO_16_9_FALLBACK_AUTO_STRATEGY
      } else {
        AspectRatioStrategy.RATIO_4_3_FALLBACK_AUTO_STRATEGY
      }

    val selector =
      ResolutionSelector.Builder()
        .setAspectRatioStrategy(aspectStrategy)
        .setResolutionStrategy(
          ResolutionStrategy(
            Size(resolution.width, resolution.height),
            ResolutionStrategy.FALLBACK_RULE_CLOSEST_LOWER_THEN_HIGHER,
          )
        )
        .build()

    val imageAnalysis =
      ImageAnalysis.Builder()
        .setResolutionSelector(selector)
        .setBackpressureStrategy(ImageAnalysis.STRATEGY_KEEP_ONLY_LATEST)
        .build()
        .also { it.setAnalyzer(executor, ::analyse) }

    val preview =
      Preview.Builder()
        .setResolutionSelector(selector)
        .build()

    val activeSp = surfaceProvider ?: headlessSurfaceProvider
    preview.setSurfaceProvider(executor, activeSp)

    try {
      cameraProvider.unbindAll()
      val cameraSelector = CameraSelector.DEFAULT_BACK_CAMERA
      cameraProvider.bindToLifecycle(lifecycleOwner, cameraSelector, imageAnalysis, preview)
      analysis = imageAnalysis
      previewUseCase = preview
      _state.value =
        _state.value.copy(
          availability = Availability.AVAILABLE,
          runState = CameraRunState.ACTIVE,
          resolution = resolution.label,
          detail = "",
        )
      logbook.info(TAG, "Camera bound at ${resolution.label} with repeating preview (${if (surfaceProvider != null) "UI" else "headless"}), cadence ${intervalMs} ms (${1000f / intervalMs} fps target)")
    } catch (e: Exception) {
      fail(e.message ?: e::class.java.simpleName)
    }
  }

  private fun analyse(image: androidx.camera.core.ImageProxy) {
    try {
      val now = System.currentTimeMillis()
      if (now - lastAnalysisAt < intervalMs) return
      lastAnalysisAt = now

      val plane = image.planes.getOrNull(0) ?: return
      var result = cachedVisibility
      if (result == null || (now - lastVisibilityAt >= 1000L)) {
        val scored =
          VisibilityEstimator.fromLuminance(
            y = plane.buffer,
            width = image.width,
            height = image.height,
            rowStride = plane.rowStride,
            pixelStride = plane.pixelStride,
            timestamp = now,
          )
        if (scored != null) {
          result = scored
          cachedVisibility = scored
          lastVisibilityAt = now
        }
      }

      val jpeg = YuvJpeg.encode(image, jpegQuality)
      analysed++
      noteFps(now)
      _state.value =
        _state.value.copy(
          visibility = result,
          framesAnalysed = analysed,
          analysisFps = currentFps(),
          resolution = "${image.width} × ${image.height}",
        )
      if (jpeg != null) {
        val payload =
          FramePayload(
            jpeg = jpeg,
            seq = ++frameSeq,
            timestamp = now,
            width = image.width,
            height = image.height,
            rotation = image.imageInfo.rotationDegrees,
          )
        if (result != null) {
          onLiveFrame?.invoke(payload, result)
        }
      }
      if (result != null) {
        onFrame?.invoke(jpeg, result)
      }

      onCameraFrame?.let { callback ->
        plane.buffer.rewind()
        val requiredCap = maxOf(plane.buffer.capacity(), plane.buffer.remaining())
        val poolIdx = yBufIndex
        yBufIndex = (yBufIndex + 1) % 3
        var targetBuf = yBufPool[poolIdx]
        if (targetBuf == null || targetBuf.capacity() < requiredCap) {
          targetBuf = java.nio.ByteBuffer.allocateDirect(requiredCap)
          yBufPool[poolIdx] = targetBuf
        }

        targetBuf.clear()
        targetBuf.put(plane.buffer)
        plane.buffer.rewind()
        targetBuf.flip()

        val monotonicTimestampNs = android.os.SystemClock.elapsedRealtimeNanos()
        val camFrame = com.minesafety.roboeye.core.model.CameraFrame(
          sequenceNumber = frameSeq,
          timestampNs = monotonicTimestampNs,
          width = image.width,
          height = image.height,
          rotationDegrees = image.imageInfo.rotationDegrees,
          yBuffer = targetBuf.asReadOnlyBuffer(),
          yRowStride = plane.rowStride,
          yPixelStride = plane.pixelStride,
          jpegData = jpeg,
          format = com.minesafety.roboeye.core.model.CameraFrame.FrameFormat.YUV_420_888,
        )
        if (frameSeq % 30L == 0L) {
          android.util.Log.i("RoboEyeGeo", "CameraController delivered frame #$frameSeq: ${image.width}x${image.height}, stride=${plane.rowStride}, cap=$requiredCap")
        }
        callback.invoke(camFrame)
      }

      // Non-blocking asynchronous perception dispatch (rate throttled to ~4 Hz)
      val minPerceptionIntervalMs = (1000f / maxPerceptionHz.coerceIn(1.0f, 10.0f)).toLong()
      if (phonePerceptionEnabled && perceptionMailbox.engine != null &&
          (now - lastPerceptionAt >= minPerceptionIntervalMs)) {
        lastPerceptionAt = now
        val rawBitmap = image.toBitmap()
        val snapshot = com.minesafety.roboeye.perception.SnapshotImageProxy(
          bitmap = rawBitmap,
          rotation = image.imageInfo.rotationDegrees,
          sequence = frameSeq,
          timestampNs = android.os.SystemClock.elapsedRealtimeNanos(),
          generation = perceptionMailbox.currentGeneration,
          frameWidth = image.width,
          frameHeight = image.height,
        )
        perceptionMailbox.submit(snapshot)
      }
    } catch (e: Exception) {
      logbook.error(TAG, "Analyser error: ${e.message ?: e::class.java.simpleName}")
    } finally {
      image.close()
    }
  }

  fun noteUploaded(at: Long) {
    uploaded++
    _state.value = _state.value.copy(framesUploaded = uploaded, lastUploadAt = at)
  }

  fun noteDropped() {
    dropped++
    _state.value = _state.value.copy(framesDropped = dropped)
  }

  fun stop() {
    perceptionMailbox.stop()
    runCatching {
      analysis?.clearAnalyzer()
      provider?.unbindAll()
    }
    headlessSurfaceProvider.release()
    analysis = null
    previewUseCase = null
    owner = null
    cachedVisibility = null
    lastVisibilityAt = 0L
    fpsWindow.clear()
    _state.value =
      _state.value.copy(
        runState = CameraRunState.STOPPED,
        analysisFps = 0f,
        visibility = null,
        detail = "",
      )
  }

  fun shutdown() {
    stop()
    perceptionMailbox.shutdown()
    executor.shutdown()
  }

  private fun fail(message: String) {
    logbook.error(TAG, "Camera error: $message")
    _state.value = _state.value.copy(runState = CameraRunState.ERROR, detail = message)
  }

  private fun noteFps(now: Long) {
    fpsWindow.addLast(now)
    while (fpsWindow.size > FPS_WINDOW) fpsWindow.removeFirst()
  }

  private fun currentFps(): Float {
    if (fpsWindow.size < 2) return 0f
    val span = fpsWindow.last() - fpsWindow.first()
    if (span <= 0) return 0f
    return (fpsWindow.size - 1) * 1000f / span
  }

  private suspend fun awaitProvider(): ProcessCameraProvider =
    suspendCancellableCoroutine { cont ->
      val future = ProcessCameraProvider.getInstance(context)
      future.addListener(
        {
          try {
            if (cont.isActive) cont.resume(future.get())
          } catch (e: Exception) {
            if (cont.isActive) cont.cancel(e)
          }
        },
        ContextCompat.getMainExecutor(context),
      )
    }

  private companion object {
    const val TAG = "Camera"
    const val FPS_WINDOW = 8
  }
}
