package com.minesafety.roboeye.vision

import kotlin.math.abs

/**
 * Assesses whether current monocular camera and scene geometry is reliable enough to trust.
 *
 * Checks visual-inertial consistency, feature tracking inlier count, vehicle tilt,
 * and turning angular velocity to produce a deterministic [GeometryTrustLevel].
 */
class SceneGeometryAssessor(
    val minTrustedInliers: Int = 12,
    val minDegradedInliers: Int = 6,
    val maxTrustedYawRateDps: Float = 16.0f,
    val maxTrustedTiltDeg: Float = 20.0f,
) {
    data class Assessment(
        val trustLevel: GeometryTrustLevel,
        val explanation: String,
    )

    fun assess(
        consistency: VisualImuConsistency,
        motionState: MotionState,
        inlierCount: Int,
        yawRateDps: Float,
        pitchDeg: Float,
        rollDeg: Float,
    ): Assessment {
        val absYaw = abs(yawRateDps)
        val absPitch = abs(pitchDeg)
        val absRoll = abs(rollDeg)

        // 1. Critical failure conditions
        if (consistency == VisualImuConsistency.CONFLICTING) {
            return Assessment(
                GeometryTrustLevel.UNTRUSTED,
                "Conflicting visual and inertial telemetry; geometry cannot be trusted",
            )
        }
        if (consistency == VisualImuConsistency.INSUFFICIENT_DATA ||
            motionState == MotionState.VISUAL_TRACKING_LOST ||
            motionState == MotionState.UNKNOWN) {
            return Assessment(
                GeometryTrustLevel.UNTRUSTED,
                "Insufficient visual or inertial data (inliers: $inlierCount, state: $motionState)",
            )
        }
        if (inlierCount < minDegradedInliers) {
            return Assessment(
                GeometryTrustLevel.UNTRUSTED,
                "Too few tracked features ($inlierCount < $minDegradedInliers)",
            )
        }

        // 2. Degraded geometry conditions
        if (absYaw > maxTrustedYawRateDps) {
            return Assessment(
                GeometryTrustLevel.DEGRADED,
                "Rapid yaw rotation (${String.format(java.util.Locale.US, "%.1f", absYaw)}°/s > ${maxTrustedYawRateDps}°/s) induces rotational perspective shear",
            )
        }
        if (absPitch > maxTrustedTiltDeg || absRoll > maxTrustedTiltDeg) {
            return Assessment(
                GeometryTrustLevel.DEGRADED,
                "Extreme chassis tilt (pitch: ${pitchDeg.toInt()}°, roll: ${rollDeg.toInt()}°) violates nominal ground plane assumption",
            )
        }
        if (inlierCount < minTrustedInliers || consistency == VisualImuConsistency.PARTIALLY_CONSISTENT) {
            return Assessment(
                GeometryTrustLevel.DEGRADED,
                "Intermediate tracking stability ($inlierCount inliers, consistency: $consistency)",
            )
        }

        // 3. Fully trusted geometry
        return Assessment(
            GeometryTrustLevel.TRUSTED,
            "Nominal geometry: consistent sensors, $inlierCount inliers, stable vehicle pose",
        )
    }
}
