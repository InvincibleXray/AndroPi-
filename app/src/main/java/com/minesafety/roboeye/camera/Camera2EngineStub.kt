package com.minesafety.roboeye.camera

import com.minesafety.roboeye.core.Availability
import com.minesafety.roboeye.core.CameraRunState
import com.minesafety.roboeye.core.CameraState
import com.minesafety.roboeye.core.model.CameraFrame
import kotlinx.coroutines.flow.Flow
import kotlinx.coroutines.flow.MutableSharedFlow
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asSharedFlow
import kotlinx.coroutines.flow.asStateFlow

/**
 * Camera2 native engine extension point for future direct manual exposure,
 * zero-copy hardware buffer access, and custom optical flow pipelines.
 */
class Camera2EngineStub : CameraEngine {

    private val _state = MutableStateFlow(
        CameraState(
            availability = Availability.WAITING,
            runState = CameraRunState.STOPPED,
            detail = "Camera2 pipeline extension point (Phase 2+)",
        )
    )
    override val state: StateFlow<CameraState> = _state.asStateFlow()

    private val _frames = MutableSharedFlow<CameraFrame>()
    override val frames: Flow<CameraFrame> = _frames.asSharedFlow()

    override fun hasPermission(): Boolean = false

    override suspend fun start(lifecycleOwner: Any, config: CameraConfig) {
        _state.value = _state.value.copy(
            detail = "Camera2 native engine will be initialized in future vision phases",
        )
    }

    override fun setPreviewSurface(surfaceProvider: Any?) {
        // Future SurfaceHolder / SurfaceTexture binding
    }

    override fun stop() {
        _state.value = _state.value.copy(runState = CameraRunState.STOPPED)
    }

    override fun shutdown() {
        stop()
    }
}
