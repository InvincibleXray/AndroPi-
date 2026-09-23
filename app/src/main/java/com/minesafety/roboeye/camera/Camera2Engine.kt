package com.minesafety.roboeye.camera

import android.Manifest
import android.content.Context
import android.content.pm.PackageManager
import android.graphics.ImageFormat
import android.hardware.camera2.CameraCaptureSession
import android.hardware.camera2.CameraCharacteristics
import android.hardware.camera2.CameraDevice
import android.hardware.camera2.CameraManager
import android.hardware.camera2.CaptureRequest
import android.media.Image
import android.media.ImageReader
import android.os.Handler
import android.os.HandlerThread
import android.view.Surface
import androidx.core.content.ContextCompat
import com.minesafety.roboeye.core.Availability
import com.minesafety.roboeye.core.CameraRunState
import com.minesafety.roboeye.core.CameraState
import com.minesafety.roboeye.core.model.CameraFrame
import kotlinx.coroutines.channels.BufferOverflow
import kotlinx.coroutines.flow.Flow
import kotlinx.coroutines.flow.MutableSharedFlow
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asSharedFlow
import kotlinx.coroutines.flow.asStateFlow
import java.nio.ByteBuffer
import java.util.concurrent.atomic.AtomicBoolean

/**
 * Native Camera2-backed implementation of [CameraEngine].
 *
 * Configured for autonomous headless/low-overhead perception:
 * - Operates with ImageReader (YUV_420_888, maxImages=2)
 * - Directly consumes Y-plane luminance buffer (no RGB conversions)
 * - Latest-frame semantics with buffer overflow drop
 * - Safe lifecycle teardown of CameraDevice, CaptureSession, and HandlerThread
 */
class Camera2Engine(
    private val context: Context,
    private val cameraId: String? = null,
) : CameraEngine {

    private val _state = MutableStateFlow(
        CameraState(
            availability = if (hasPermission()) Availability.WAITING else Availability.NO_PERMISSION,
            runState = CameraRunState.STOPPED,
            detail = "Camera2 Engine Initialized",
        )
    )
    override val state: StateFlow<CameraState> = _state.asStateFlow()

    private val _frames = MutableSharedFlow<CameraFrame>(
        replay = 0,
        extraBufferCapacity = 2,
        onBufferOverflow = BufferOverflow.DROP_OLDEST,
    )
    override val frames: Flow<CameraFrame> = _frames.asSharedFlow()

    private val cameraManager = context.getSystemService(Context.CAMERA_SERVICE) as? CameraManager

    private var backgroundThread: HandlerThread? = null
    private var backgroundHandler: Handler? = null

    private var cameraDevice: CameraDevice? = null
    private var captureSession: CameraCaptureSession? = null
    private var imageReader: ImageReader? = null
    private var previewSurface: Surface? = null

    private var currentConfig: CameraConfig = CameraConfig.LOW_END_DEFAULT
    private var frameSeq: Long = 0L
    private val isStopping = AtomicBoolean(false)

    override fun hasPermission(): Boolean =
        ContextCompat.checkSelfPermission(context, Manifest.permission.CAMERA) ==
            PackageManager.PERMISSION_GRANTED

    private fun startBackgroundThread() {
        if (backgroundThread == null) {
            val thread = HandlerThread("RoverCamera2Thread")
            thread.start()
            backgroundThread = thread
            backgroundHandler = Handler(thread.looper)
        }
    }

    private fun stopBackgroundThread() {
        backgroundThread?.quitSafely()
        try {
            backgroundThread?.join(500L)
        } catch (_: InterruptedException) {
        }
        backgroundThread = null
        backgroundHandler = null
    }

    override suspend fun start(lifecycleOwner: Any, config: CameraConfig) {
        if (!hasPermission()) {
            _state.value = _state.value.copy(
                availability = Availability.NO_PERMISSION,
                runState = CameraRunState.STOPPED,
                detail = "Camera permission not granted for Camera2",
            )
            return
        }

        if (cameraManager == null) {
            _state.value = _state.value.copy(
                availability = Availability.UNAVAILABLE,
                runState = CameraRunState.STOPPED,
                detail = "CameraManager unavailable",
            )
            return
        }

        currentConfig = config
        isStopping.set(false)
        startBackgroundThread()

        _state.value = _state.value.copy(
            runState = CameraRunState.STARTING,
            detail = "Opening Camera2 device",
        )

        val targetId = cameraId ?: Camera2CapabilityHelper.probeCamera2(cameraManager).cameraId
        if (targetId == null) {
            _state.value = _state.value.copy(
                availability = Availability.UNAVAILABLE,
                runState = CameraRunState.STOPPED,
                detail = "No compatible Camera2 camera found",
            )
            return
        }

        try {
            // Configure ImageReader for YUV_420_888 with maxImages = 2
            val reader = ImageReader.newInstance(
                config.targetWidth,
                config.targetHeight,
                ImageFormat.YUV_420_888,
                2
            )
            reader.setOnImageAvailableListener(::onImageAvailable, backgroundHandler)
            imageReader = reader

            cameraManager.openCamera(
                targetId,
                object : CameraDevice.StateCallback() {
                    override fun onOpened(camera: CameraDevice) {
                        if (isStopping.get()) {
                            camera.close()
                            return
                        }
                        cameraDevice = camera
                        createCaptureSession(camera)
                    }

                    override fun onDisconnected(camera: CameraDevice) {
                        camera.close()
                        cameraDevice = null
                        _state.value = _state.value.copy(
                            runState = CameraRunState.STOPPED,
                            detail = "Camera2 device disconnected",
                        )
                    }

                    override fun onError(camera: CameraDevice, error: Int) {
                        camera.close()
                        cameraDevice = null
                        _state.value = _state.value.copy(
                            availability = Availability.UNAVAILABLE,
                            runState = CameraRunState.ERROR,
                            detail = "Camera2 device error: $error",
                        )
                    }
                },
                backgroundHandler
            )
        } catch (e: SecurityException) {
            _state.value = _state.value.copy(
                availability = Availability.NO_PERMISSION,
                runState = CameraRunState.ERROR,
                detail = "Camera permission error: ${e.message}",
            )
        } catch (e: Exception) {
            _state.value = _state.value.copy(
                availability = Availability.UNAVAILABLE,
                runState = CameraRunState.ERROR,
                detail = "Failed to open Camera2: ${e.message}",
            )
        }
    }

    private fun createCaptureSession(camera: CameraDevice) {
        val reader = imageReader ?: return
        val surfaces = mutableListOf<Surface>(reader.surface)
        previewSurface?.let { surfaces.add(it) }

        try {
            camera.createCaptureSession(
                surfaces,
                object : CameraCaptureSession.StateCallback() {
                    override fun onConfigured(session: CameraCaptureSession) {
                        if (isStopping.get() || cameraDevice == null) return
                        captureSession = session
                        startRepeatingRequest(session)
                        _state.value = _state.value.copy(
                            availability = Availability.AVAILABLE,
                            runState = CameraRunState.ACTIVE,
                            resolution = "${currentConfig.targetWidth} × ${currentConfig.targetHeight}",
                            detail = "Camera2 active (${currentConfig.targetWidth}x${currentConfig.targetHeight})",
                        )
                    }

                    override fun onConfigureFailed(session: CameraCaptureSession) {
                        _state.value = _state.value.copy(
                            availability = Availability.UNAVAILABLE,
                            runState = CameraRunState.ERROR,
                            detail = "Camera2 capture session configuration failed",
                        )
                    }
                },
                backgroundHandler
            )
        } catch (e: Exception) {
            _state.value = _state.value.copy(
                availability = Availability.UNAVAILABLE,
                runState = CameraRunState.ERROR,
                detail = "Create capture session exception: ${e.message}",
            )
        }
    }

    private fun startRepeatingRequest(session: CameraCaptureSession) {
        val device = cameraDevice ?: return
        val reader = imageReader ?: return

        try {
            val template = if (previewSurface != null) {
                CameraDevice.TEMPLATE_PREVIEW
            } else {
                CameraDevice.TEMPLATE_RECORD
            }
            val requestBuilder = device.createCaptureRequest(template).apply {
                addTarget(reader.surface)
                previewSurface?.let { addTarget(it) }
                set(CaptureRequest.CONTROL_MODE, CaptureRequest.CONTROL_MODE_AUTO)
                set(CaptureRequest.CONTROL_AF_MODE, CaptureRequest.CONTROL_AF_MODE_CONTINUOUS_PICTURE)
            }
            session.setRepeatingRequest(requestBuilder.build(), null, backgroundHandler)
        } catch (e: Exception) {
            _state.value = _state.value.copy(
                availability = Availability.UNAVAILABLE,
                runState = CameraRunState.ERROR,
                detail = "Set repeating request failed: ${e.message}",
            )
        }
    }

    private fun onImageAvailable(reader: ImageReader) {
        var image: Image? = null
        try {
            image = reader.acquireLatestImage() ?: return
            val plane = image.planes.getOrNull(0) ?: return

            // Make an independent copy of luminance buffer so downstream processing is safe
            val yBuffer = ByteBuffer.allocate(plane.buffer.remaining())
            plane.buffer.rewind()
            yBuffer.put(plane.buffer)
            plane.buffer.rewind()
            yBuffer.flip()

            val now = System.currentTimeMillis()
            val frame = CameraFrame(
                sequenceNumber = ++frameSeq,
                timestampNs = image.timestamp.takeIf { it > 0 } ?: (now * 1_000_000L),
                width = image.width,
                height = image.height,
                rotationDegrees = 0,
                yBuffer = yBuffer,
                yRowStride = plane.rowStride,
                yPixelStride = plane.pixelStride,
                format = CameraFrame.FrameFormat.YUV_420_888,
            )
            _frames.tryEmit(frame)
        } catch (_: IllegalStateException) {
            // Buffer already closed or HAL dropped frame
        } catch (_: Exception) {
        } finally {
            try {
                image?.close()
            } catch (_: Exception) {
            }
        }
    }

    override fun setPreviewSurface(surfaceProvider: Any?) {
        val newSurface = surfaceProvider as? Surface
        if (previewSurface != newSurface) {
            previewSurface = newSurface
            val session = captureSession
            val device = cameraDevice
            if (device != null && session != null && _state.value.runState == CameraRunState.ACTIVE) {
                // Dynamic session update
                try {
                    session.stopRepeating()
                    session.close()
                    captureSession = null
                    createCaptureSession(device)
                } catch (_: Exception) {
                }
            }
        }
    }

    override fun stop() {
        isStopping.set(true)
        try {
            captureSession?.stopRepeating()
            captureSession?.close()
        } catch (_: Exception) {
        }
        captureSession = null

        try {
            cameraDevice?.close()
        } catch (_: Exception) {
        }
        cameraDevice = null

        try {
            imageReader?.close()
        } catch (_: Exception) {
        }
        imageReader = null

        _state.value = _state.value.copy(
            runState = CameraRunState.STOPPED,
            detail = "Camera2 stopped",
        )
    }

    override fun shutdown() {
        stop()
        stopBackgroundThread()
    }
}
