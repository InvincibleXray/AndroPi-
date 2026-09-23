package com.minesafety.roboeye.camera

import android.content.Context
import com.minesafety.roboeye.core.CameraRunState
import com.minesafety.roboeye.core.CameraState
import com.minesafety.roboeye.core.Logbook
import com.minesafety.roboeye.core.model.CameraFrame
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.SupervisorJob
import kotlinx.coroutines.flow.Flow
import kotlinx.coroutines.flow.MutableSharedFlow
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asSharedFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.launch

/**
 * Adaptive camera engine that selects between Camera2 and CameraX based on
 * user configuration and hardware capabilities.
 *
 * Defaults to CameraX for stability. When Camera2 is requested, it verifies
 * hardware support (rejecting LEGACY HALs) and falls back safely to CameraX if
 * any error occurs during initialization or streaming.
 */
class AdaptiveCameraEngine(
    private val context: Context,
    val cameraXEngine: CameraXEngine,
    val camera2Engine: CameraEngine? = null,
    private val logbook: Logbook? = null,
) : CameraEngine {

    companion object {
        private const val TAG = "AdaptiveCameraEngine"
    }

    private val scope = CoroutineScope(Dispatchers.Main.immediate + SupervisorJob())

    private val _state = MutableStateFlow(cameraXEngine.state.value)
    override val state: StateFlow<CameraState> = _state.asStateFlow()

    private val _frames = MutableSharedFlow<CameraFrame>(replay = 0, extraBufferCapacity = 2)
    override val frames: Flow<CameraFrame> = _frames.asSharedFlow()

    private var activeEngine: CameraEngine = cameraXEngine
    var usingCamera2: Boolean = false
        private set

    init {
        // Forward frames from active engine
        scope.launch {
            cameraXEngine.frames.collect { frame ->
                if (activeEngine === cameraXEngine) {
                    _frames.tryEmit(frame)
                }
            }
        }
        camera2Engine?.let { c2 ->
            scope.launch {
                c2.frames.collect { frame ->
                    if (activeEngine === c2) {
                        _frames.tryEmit(frame)
                    }
                }
            }
        }
    }

    override fun hasPermission(): Boolean = activeEngine.hasPermission()

    override suspend fun start(lifecycleOwner: Any, config: CameraConfig) {
        if (config.useCamera2 && camera2Engine != null) {
            val capability = Camera2CapabilityHelper.probeCamera2(context, config.targetWidth, config.targetHeight)
            if (capability.isSupported) {
                logbook?.info(TAG, "Attempting Camera2 start with ${capability.hardwareLevelName} HAL")
                try {
                    activeEngine = camera2Engine
                    usingCamera2 = true
                    camera2Engine.start(lifecycleOwner, config)

                    // Verify Camera2 did not immediately transition into STOPPED / ERROR state
                    if (camera2Engine.state.value.runState == CameraRunState.ACTIVE ||
                        camera2Engine.state.value.runState == CameraRunState.STARTING) {
                        forwardState(camera2Engine)
                        return
                    } else {
                        logbook?.warn(TAG, "Camera2 failed to start (${camera2Engine.state.value.detail}). Falling back to CameraX.")
                    }
                } catch (e: Exception) {
                    logbook?.warn(TAG, "Camera2 exception during start: ${e.message}. Falling back to CameraX.")
                }
            } else {
                logbook?.info(TAG, "Camera2 not viable (${capability.reason}). Using CameraX.")
            }
        }

        // Fallback or default path: CameraX
        activeEngine = cameraXEngine
        usingCamera2 = false
        cameraXEngine.start(lifecycleOwner, config)
        forwardState(cameraXEngine)
    }

    private fun forwardState(engine: CameraEngine) {
        scope.launch {
            engine.state.collect { s ->
                if (activeEngine === engine) {
                    _state.value = s
                }
            }
        }
    }

    override fun setPreviewSurface(surfaceProvider: Any?) {
        activeEngine.setPreviewSurface(surfaceProvider)
    }

    override fun stop() {
        activeEngine.stop()
        _state.value = activeEngine.state.value
    }

    override fun shutdown() {
        camera2Engine?.shutdown()
        cameraXEngine.shutdown()
    }
}
