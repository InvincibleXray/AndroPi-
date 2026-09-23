package com.minesafety.roboeye.fusion

import kotlin.math.abs

enum class ReadingStatus {
    VALID,
    INVALID,
    STALE,
    UNAVAILABLE,
}

data class ValidatedRange(
    val distanceM: Float?,
    val status: ReadingStatus,
    val confidence: Float,
    val ageMs: Long,
) {
    val isValid: Boolean get() = status == ReadingStatus.VALID
}

/**
 * Validates distance sensors, IMU observations, and encoders against physical limits,
 * freshness windows, and impossible jump discontinuities.
 */
class SensorValidator(
    val maxRangeFreshnessMs: Long = 500L,
    val maxImuFreshnessMs: Long = 250L,
    val maxVisionFreshnessMs: Long = 500L,
    val maxJumpM: Float = 2.5f,
    val maxJumpIntervalMs: Long = 100L,
) {
    private val previousRanges = HashMap<String, Pair<Float, Long>>()

    fun validateRange(
        sensorId: String,
        measuredM: Float?,
        readingTimestampMs: Long,
        nowMs: Long = System.currentTimeMillis(),
        mount: DistanceSensorMount? = null,
    ): ValidatedRange {
        if (measuredM == null) {
            return ValidatedRange(null, ReadingStatus.UNAVAILABLE, 0.0f, 0L)
        }

        val ageMs = (nowMs - readingTimestampMs).coerceAtLeast(0L)
        if (ageMs > maxRangeFreshnessMs) {
            return ValidatedRange(measuredM, ReadingStatus.STALE, 0.0f, ageMs)
        }

        val minM = mount?.rangeMinM ?: 0.02f
        val maxM = mount?.rangeMaxM ?: 4.5f

        if (!measuredM.isFinite() || measuredM < minM || measuredM > maxM) {
            return ValidatedRange(null, ReadingStatus.INVALID, 0.0f, ageMs)
        }

        // Jump discontinuity check
        val prev = previousRanges[sensorId]
        if (prev != null) {
            val (prevDist, prevTime) = prev
            val dt = nowMs - prevTime
            if (dt in 1..maxJumpIntervalMs && abs(measuredM - prevDist) > maxJumpM) {
                // Reject anomalous acoustic spike / multi-path reflection
                return ValidatedRange(prevDist, ReadingStatus.INVALID, 0.1f, ageMs)
            }
        }

        previousRanges[sensorId] = Pair(measuredM, nowMs)

        // Calculate freshness confidence: 1.0 down to 0.5 near expiration
        val freshnessFactor = (1.0f - (ageMs.toFloat() / maxRangeFreshnessMs) * 0.5f).coerceIn(0.5f, 1.0f)
        return ValidatedRange(measuredM, ReadingStatus.VALID, freshnessFactor, ageMs)
    }

    fun isImuFresh(timestampMs: Long, nowMs: Long = System.currentTimeMillis()): Boolean {
        val age = (nowMs - timestampMs).coerceAtLeast(0L)
        return age <= maxImuFreshnessMs
    }

    fun isVisionFresh(timestampMs: Long, nowMs: Long = System.currentTimeMillis()): Boolean {
        val age = (nowMs - timestampMs).coerceAtLeast(0L)
        return age <= maxVisionFreshnessMs
    }

    fun reset() {
        previousRanges.clear()
    }
}
