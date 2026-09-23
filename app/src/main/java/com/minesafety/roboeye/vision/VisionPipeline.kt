package com.minesafety.roboeye.vision

import com.minesafety.roboeye.core.model.CameraFrame
import com.minesafety.roboeye.core.model.FreeSpaceObservation
import com.minesafety.roboeye.core.model.ObstacleObservation

/**
 * Output packet produced by a vision pipeline execution pass.
 */
data class VisionOutput(
    val frameSequence: Long,
    val timestampNs: Long,
    val obstacles: List<ObstacleObservation> = emptyList(),
    val freeSpace: FreeSpaceObservation = FreeSpaceObservation(),
    val visibilityScore: Float? = null,
    val processingTimeMs: Long = 0L,
)

/**
 * High-level Vision Pipeline interface.
 *
 * Consumes decoupled [CameraFrame] inputs and emits structured [VisionOutput] observations.
 * Implementation of specific algorithms is deferred to future vision phases.
 */
interface VisionPipeline {
    /** Whether this vision pipeline is active and ready to process frames. */
    val isReady: Boolean

    /** Processes a single camera frame and outputs spatial observations. */
    fun processFrame(frame: CameraFrame): VisionOutput
}
