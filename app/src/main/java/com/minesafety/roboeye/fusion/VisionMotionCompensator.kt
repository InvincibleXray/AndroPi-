package com.minesafety.roboeye.fusion

import com.minesafety.roboeye.core.ImuReading
import kotlin.math.abs

/**
 * Compensates visual optical flow against vehicle rotational motion detected by the phone IMU.
 *
 * Prevents false collision alerts: when a rover executes a turn or pivot, scene features
 * undergo strong lateral flow that can produce false optical flow convergence or expansion.
 */
class VisionMotionCompensator(
    val rotationThresholdDps: Float = 15.0f,
    val suppressionRangeDps: Float = 30.0f,
) {

    data class CompensationResult(
        val isRotationalMotion: Boolean,
        val confidenceMultiplier: Float,
        val yawRateDps: Float,
        val explanation: String,
    )

    fun evaluate(imu: ImuReading?): CompensationResult {
        if (imu == null) {
            // IMU unavailable: cannot perform compensation, preserve baseline vision confidence
            return CompensationResult(
                isRotationalMotion = false,
                confidenceMultiplier = 1.0f,
                yawRateDps = 0.0f,
                explanation = "IMU unavailable; raw visual confidence used",
            )
        }

        val yawRateDps = when {
            imu.gyroZ != null -> abs(Math.toDegrees(imu.gyroZ.toDouble()).toFloat())
            imu.yawRateDps != 0.0f -> abs(imu.yawRateDps)
            else -> 0.0f
        }

        if (yawRateDps <= rotationThresholdDps) {
            return CompensationResult(
                isRotationalMotion = false,
                confidenceMultiplier = 1.0f,
                yawRateDps = yawRateDps,
                explanation = "Nominal translational motion (|yawRate| <= ${rotationThresholdDps}°/s)",
            )
        }

        // Damping factor smoothly scales from 1.0 down to 0.0 over suppressionRangeDps
        val excess = yawRateDps - rotationThresholdDps
        val multiplier = (1.0f - (excess / suppressionRangeDps)).coerceIn(0.0f, 1.0f)
        val isRotational = multiplier < 0.5f

        return CompensationResult(
            isRotationalMotion = isRotational,
            confidenceMultiplier = multiplier,
            yawRateDps = yawRateDps,
            explanation = "Vehicle turning detected (${String.format(java.util.Locale.US, "%.1f", yawRateDps)}°/s); visual flow confidence damped by factor $multiplier",
        )
    }

    /**
     * Damps visual TTC confidence based on vehicle rotation.
     * If rotation is severe (multiplier == 0.0), TTC is suppressed (null).
     */
    fun compensateTtc(
        rawTtcSec: Float?,
        rawConfidence: Float,
        imu: ImuReading?
    ): Pair<Float?, Float> {
        val comp = evaluate(imu)
        if (rawTtcSec == null) return Pair(null, 0.0f)

        val dampedConfidence = rawConfidence * comp.confidenceMultiplier
        val compensatedTtc = if (comp.confidenceMultiplier < 0.1f) null else rawTtcSec
        return Pair(compensatedTtc, dampedConfidence)
    }
}
