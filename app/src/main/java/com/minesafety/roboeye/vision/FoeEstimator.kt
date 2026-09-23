package com.minesafety.roboeye.vision

data class FoeResult(
    /** Focus of Expansion horizontal coordinate in normalized [0, 1] frame space. */
    val foeX: Float = 0.5f,
    /** Focus of Expansion vertical coordinate in normalized [0, 1] frame space. */
    val foeY: Float = 0.5f,
    val confidence: Float = 0.0f,
    val isDivergent: Boolean = false,
    val timestampNs: Long = System.nanoTime(),
)

/**
 * Focus of Expansion (FOE) boundary interface for heading and optical flow expansion estimation.
 */
interface FoeEstimator {
    fun estimateFoe(flow: OpticalFlowResult): FoeResult
}
