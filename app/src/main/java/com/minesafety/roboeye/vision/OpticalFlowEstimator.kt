package com.minesafety.roboeye.vision

import com.minesafety.roboeye.core.model.CameraFrame

data class FlowVector(
    val prevX: Float,
    val prevY: Float,
    val currX: Float,
    val currY: Float,
    val confidence: Float = 1.0f,
    val isValid: Boolean = true,
) {
    val x: Float get() = prevX
    val y: Float get() = prevY
    val vx: Float get() = currX - prevX
    val vy: Float get() = currY - prevY
    val dx: Float get() = vx
    val dy: Float get() = vy
    val length: Float get() = kotlin.math.hypot(dx, dy)
}

data class OpticalFlowResult(
    val vectors: List<FlowVector> = emptyList(),
    val averageDivergence: Float = 0.0f,
    val timestampNs: Long = System.nanoTime(),
)

/**
 * Lucas-Kanade sparse optical flow boundary interface.
 */
interface OpticalFlowEstimator {
    fun trackFlow(previousFrame: CameraFrame, currentFrame: CameraFrame): OpticalFlowResult

    fun track(
        prevFrame: CameraFrame,
        currFrame: CameraFrame,
        prevPoints: List<FeaturePoint>
    ): List<FlowVector> = emptyList()
}
