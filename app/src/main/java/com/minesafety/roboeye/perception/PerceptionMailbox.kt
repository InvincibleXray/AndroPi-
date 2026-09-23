package com.minesafety.roboeye.perception

import android.graphics.Bitmap
import android.graphics.PixelFormat
import android.graphics.Rect
import androidx.camera.core.ImageInfo
import androidx.camera.core.ImageProxy
import java.util.concurrent.ExecutorService
import java.util.concurrent.Executors
import java.util.concurrent.atomic.AtomicLong

/**
 * Snapshot representation of a camera frame specifically for asynchronous perception.
 *
 * Implements [ImageProxy] so that downstream detectors ([YoloDetector] and [SsdMobileNetDetector])
 * receive genuine frame dimensions, rotation, and [toBitmap] access without requiring any detector
 * modifications.
 *
 * Owns its [Bitmap] exclusively and guarantees prompt, safe recycling when closed.
 */
open class SnapshotImageProxy(
    private val bitmap: Bitmap?,
    private val rotation: Int,
    val sequence: Long,
    val timestampNs: Long,
    val generation: Long,
    private val frameWidth: Int = bitmap?.width ?: 640,
    private val frameHeight: Int = bitmap?.height ?: 360,
    private val onClose: (() -> Unit)? = null,
) : ImageProxy {

    private var isClosed = false

    override fun close() {
        if (!isClosed) {
            isClosed = true
            bitmap?.let {
                if (!it.isRecycled) {
                    it.recycle()
                }
            }
            onClose?.invoke()
        }
    }

    override fun getCropRect(): Rect = Rect(0, 0, width, height)

    override fun setCropRect(rect: Rect?) {}

    override fun getFormat(): Int = PixelFormat.RGBA_8888

    override fun getHeight(): Int = frameHeight

    override fun getWidth(): Int = frameWidth

    override fun getPlanes(): Array<ImageProxy.PlaneProxy> = emptyArray()

    override fun getImageInfo(): ImageInfo = object : ImageInfo {
        override fun getTagBundle(): androidx.camera.core.impl.TagBundle =
            androidx.camera.core.impl.TagBundle.emptyBundle()

        override fun getTimestamp(): Long = timestampNs

        override fun getRotationDegrees(): Int = rotation

        override fun populateExifData(builder: androidx.camera.core.impl.utils.ExifData.Builder) {}
    }

    override fun getImage(): android.media.Image? = null

    override fun toBitmap(): Bitmap =
        bitmap ?: Bitmap.createBitmap(frameWidth, frameHeight, Bitmap.Config.ARGB_8888)
}

/**
 * Bounded latest-frame mailbox and dedicated execution context for AI perception.
 *
 * Workload Bounds:
 * - ACTIVE: at most 1 inference executing at any given time.
 * - PENDING: at most 1 latest frame waiting to be processed.
 *
 * When an active inference is in progress, any incoming eligible frame replaces the existing
 * pending frame, immediately recycling the older pending frame to prevent memory accumulation.
 *
 * The Camera analysis executor NEVER waits on this mailbox, ensuring zero jitter or starvation
 * for CameraX and the Geometric Vision pipeline.
 */
class PerceptionMailbox(
    private val executor: ExecutorService = Executors.newSingleThreadExecutor { runnable ->
        Thread(runnable, "PerceptionWorker").apply {
            priority = Thread.NORM_PRIORITY - 1
        }
    }
) {
    private val lock = Any()

    // At most 1 pending frame
    private var pendingFrame: SnapshotImageProxy? = null

    // At most 1 active worker execution
    private var isWorkerActive = false

    // Monotonic generation counter to reject in-flight work on disable/stop/restart
    private val generation = AtomicLong(0L)

    // Tracks accepted frame sequence to guarantee strictly monotonic result ordering
    private var lastAcceptedSequence = 0L

    @Volatile var isEnabled: Boolean = false
    @Volatile var sensitivity: Float = 0.35f
    var engine: PerceptionEngine? = null
    var onResult: ((PerceptionFrameResult) -> Unit)? = null
    var onError: ((Throwable) -> Unit)? = null
    var onFrameSkipped: (() -> Unit)? = null

    val currentGeneration: Long get() = generation.get()

    /**
     * Non-blocking submission from the camera analysis thread.
     *
     * Takes ownership of [frame]. If another pending frame already exists, the older frame
     * is immediately closed and its bitmap recycled.
     */
    fun submit(frame: SnapshotImageProxy) {
        synchronized(lock) {
            if (!isEnabled || engine == null) {
                frame.close()
                return
            }

            // Bounded to 1 pending frame: replace any existing pending frame
            val oldPending = pendingFrame
            pendingFrame = frame
            if (oldPending != null) {
                oldPending.close()
                onFrameSkipped?.invoke()
            }

            // If the worker loop is not currently active, launch it
            if (!isWorkerActive) {
                isWorkerActive = true
                executor.execute {
                    drainMailbox()
                }
            }
        }
    }

    private fun drainMailbox() {
        while (true) {
            val frameToProcess: SnapshotImageProxy
            val activeEngine: PerceptionEngine
            val activeSensitivity: Float
            val targetGeneration: Long

            synchronized(lock) {
                val currentPending = pendingFrame
                val currentEngine = engine
                if (!isEnabled || currentPending == null || currentEngine == null) {
                    isWorkerActive = false
                    return
                }

                // Pop pending frame -> becomes the single active frame
                frameToProcess = currentPending
                pendingFrame = null
                activeEngine = currentEngine
                activeSensitivity = sensitivity
                targetGeneration = generation.get()
            }

            // ACTIVE = 1 right here (inference executes outside synchronized block)
            try {
                if (frameToProcess.generation == targetGeneration) {
                    val result = activeEngine.processFrame(frameToProcess, activeSensitivity)

                    synchronized(lock) {
                        // Reject stale results if perception was disabled, reset, or newer frame accepted
                        if (isEnabled &&
                            frameToProcess.generation == generation.get() &&
                            frameToProcess.sequence > lastAcceptedSequence
                        ) {
                            lastAcceptedSequence = frameToProcess.sequence
                            onResult?.invoke(result)
                        }
                    }
                }
            } catch (t: Throwable) {
                onError?.invoke(t)
            } finally {
                frameToProcess.close()
            }
        }
    }

    /**
     * Disables perception, purges pending frames, and increments generation to invalidate in-flight work.
     */
    fun onDisabled() {
        synchronized(lock) {
            isEnabled = false
            generation.incrementAndGet()
            val old = pendingFrame
            pendingFrame = null
            old?.close()
        }
    }

    /**
     * Enables perception under a fresh generation token.
     */
    fun onEnabled() {
        synchronized(lock) {
            generation.incrementAndGet()
            lastAcceptedSequence = 0L
            val old = pendingFrame
            pendingFrame = null
            old?.close()
            isEnabled = true
        }
    }

    /**
     * Lifecycle stop: purges pending frame and increments generation.
     */
    fun stop() {
        synchronized(lock) {
            generation.incrementAndGet()
            val old = pendingFrame
            pendingFrame = null
            old?.close()
        }
    }

    /**
     * Clean shutdown of perception executor.
     */
    fun shutdown() {
        stop()
        executor.shutdown()
    }
}
