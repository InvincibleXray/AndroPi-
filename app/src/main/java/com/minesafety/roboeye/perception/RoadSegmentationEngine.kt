package com.minesafety.roboeye.perception

import com.minesafety.roboeye.core.Units

/**
 * Road-perception seam.
 */
class RoadSegmentationEngine(
    /** Age past which a supplied result stops being presented as current. */
    val staleThresholdMs: Long = 1500L,
) {

    @Volatile private var cachedResult: RoadSegmentationResult = RoadSegmentationResult.unavailable()

    /**
     * The most recent road result, with staleness re-evaluated against [nowMs].
     */
    fun getLatestResult(nowMs: Long = System.currentTimeMillis()): RoadSegmentationResult {
        val current = cachedResult
        if (!current.hasValidRoad) return current
        val isStale = (nowMs - current.timestampMs) > staleThresholdMs
        return if (isStale != current.isStale) current.copy(isStale = isStale) else current
    }

    /** Publishes a result produced from real pixels. */
    fun updateResult(result: RoadSegmentationResult) {
        cachedResult = result
    }

    /** Drops any published result back to "not perceived". */
    fun reset() {
        cachedResult = RoadSegmentationResult.unavailable()
    }
}

/**
 * Output of a road-perception stage.
 */
data class RoadSegmentationResult(
    val hasValidRoad: Boolean,
    val roadConfidence: Float,
    val leftBoundaryBearingDeg: Float,
    val rightBoundaryBearingDeg: Float,
    val drivableWidthAtRoverM: Float,
    val roadCenterlineOffsetDeg: Float,
    val timestampMs: Long,
    val isStale: Boolean,
) {
    /**
     * Drivable width at [distanceM] ahead, or 0 when no road was perceived.
     */
    fun getDrivableWidthAtDistance(distanceM: Float): Float {
        if (!hasValidRoad) return 0.0f
        val d = distanceM.coerceIn(0.5f, 50.0f)
        return Units.round1(drivableWidthAtRoverM * (1.0f + 0.015f * d))
    }

    companion object {
        /** Default unavailable state */
        fun unavailable(nowMs: Long = System.currentTimeMillis()) = RoadSegmentationResult(
            hasValidRoad = false,
            roadConfidence = 0.0f,
            leftBoundaryBearingDeg = 0.0f,
            rightBoundaryBearingDeg = 0.0f,
            drivableWidthAtRoverM = 0.0f,
            roadCenterlineOffsetDeg = 0.0f,
            timestampMs = nowMs,
            isStale = false,
        )
    }
}
