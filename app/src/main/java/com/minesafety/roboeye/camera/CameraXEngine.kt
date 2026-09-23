package com.minesafety.roboeye.camera

import androidx.camera.core.Preview
import androidx.lifecycle.LifecycleOwner
import com.minesafety.roboeye.core.CameraResolution
import com.minesafety.roboeye.core.CameraState
import com.minesafety.roboeye.core.model.CameraFrame
import com.minesafety.roboeye.sensors.CameraController
import kotlinx.coroutines.channels.BufferOverflow
import kotlinx.coroutines.flow.Flow
import kotlinx.coroutines.flow.MutableSharedFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asSharedFlow

/**
 * CameraX-backed implementation of [CameraEngine].
 *
 * Adapts the verified, production-tested [CameraController] behind the unified
 * Rover Brain [CameraEngine] contract without modifying or breaking existing behavior.
 */
class CameraXEngine(
    val controller: CameraController,
) : CameraEngine {

    override val state: StateFlow<CameraState> = controller.state

    private val _frames = MutableSharedFlow<CameraFrame>(
        replay = 0,
        extraBufferCapacity = 2,
        onBufferOverflow = BufferOverflow.DROP_OLDEST,
    )
    override val frames: Flow<CameraFrame> = _frames.asSharedFlow()

    init {
        // Wire the frame callback to emit decoupled CameraFrame models with direct Y-buffer
        controller.onCameraFrame = { frame ->
            _frames.tryEmit(frame)
        }
        controller.onLiveFrame = { payload, _ ->
            // Fallback if onCameraFrame wasn't invoked
            if (_frames.replayCache.isEmpty()) {
                val frame = CameraFrame(
                    sequenceNumber = payload.seq,
                    timestampNs = payload.timestamp * 1_000_000L,
                    width = payload.width,
                    height = payload.height,
                    rotationDegrees = payload.rotation,
                    jpegData = payload.jpeg,
                    format = CameraFrame.FrameFormat.JPEG_ONLY,
                )
                _frames.tryEmit(frame)
            }
        }
    }

    override fun hasPermission(): Boolean = controller.hasPermission()

    override suspend fun start(lifecycleOwner: Any, config: CameraConfig) {
        val owner = lifecycleOwner as? LifecycleOwner
            ?: throw IllegalArgumentException("CameraXEngine requires an androidx.lifecycle.LifecycleOwner")

        val res = when {
            config.targetWidth >= 1280 -> CameraResolution.HD720
            config.targetWidth == 640 && config.targetHeight == 360 -> CameraResolution.NHD
            config.targetWidth >= 640 -> CameraResolution.VGA
            else -> CameraResolution.QVGA
        }

        controller.start(
            lifecycleOwner = owner,
            cameraResolution = res,
            frameIntervalMs = config.intervalMs,
            quality = config.jpegQuality,
        )
    }

    override fun setPreviewSurface(surfaceProvider: Any?) {
        val sp = surfaceProvider as? Preview.SurfaceProvider
        controller.setPreviewSurface(sp)
    }

    override fun stop() {
        controller.stop()
    }

    override fun shutdown() {
        controller.shutdown()
    }
}
