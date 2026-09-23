package com.minesafety.roboeye.camera

import com.minesafety.roboeye.core.CameraState
import com.minesafety.roboeye.core.model.CameraFrame
import kotlinx.coroutines.flow.Flow
import kotlinx.coroutines.flow.StateFlow

/**
 * Universal camera engine abstraction for Rover Brain.
 *
 * Isolates downstream vision and robotics pipelines from specific camera APIs
 * (CameraX vs Camera2) and Android UI rendering.
 */
interface CameraEngine {
    /** Current camera health and operational state. */
    val state: StateFlow<CameraState>

    /** Hot flow of camera frames emitted at the configured cadence. */
    val frames: Flow<CameraFrame>

    /** Checks whether the required camera permission is granted. */
    fun hasPermission(): Boolean

    /**
     * Starts the camera pipeline.
     *
     * @param lifecycleOwner Android LifecycleOwner to bind to (e.g. Activity or Service).
     * @param config Target resolution, cadence, and preview parameters.
     */
    suspend fun start(lifecycleOwner: Any, config: CameraConfig)

    /**
     * Attaches an optional preview surface for human/debug view.
     * Passing null transitions the engine into headless autonomous mode with zero preview overhead.
     */
    fun setPreviewSurface(surfaceProvider: Any?)

    /** Stops camera streaming and unbinds capture sessions. */
    fun stop()

    /** Releases all thread executors and native camera resources. */
    fun shutdown()
}
