package com.minesafety.roboeye.vision

data class TtcResult(
    /** Minimum Time-To-Collision in seconds across the field of view. */
    val minTtcSec: Float? = null,
    val criticalSectorBearingDeg: Float? = null,
    val isImminentCollision: Boolean = false,
    val timestampNs: Long = System.nanoTime(),
)

/**
 * Optical flow divergence-based Time-To-Collision (TTC) boundary interface.
 */
interface TtcEstimator {
    fun estimateTtc(flow: OpticalFlowResult, foe: FoeResult): TtcResult
}
