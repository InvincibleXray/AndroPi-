package com.minesafety.roboeye.localization

import kotlin.math.roundToInt

/**
 * Observable scale validity state for monocular visual motion estimation.
 */
enum class MonocularScaleState {
    /** Absolute scale cannot be verified; motion is normalized relative units. */
    SCALE_UNKNOWN,
    /** Absolute scale estimated via calibrated camera mounting height and ground pitch declination. */
    SCALE_ESTIMATED,
    /** Scale estimation degraded due to tilt anomalies, feature loss, or sensor conflict. */
    SCALE_UNRELIABLE,
}

/**
 * Operational tracking health state for visual-inertial localization.
 */
enum class TrackingQuality {
    /** Initializing at local origin (0, 0, 0°). */
    INITIALIZING,
    /** Nominal tracking with consistent visual features and IMU telemetry. */
    TRACKING,
    /** Degraded tracking due to sparse features, high rotation, or minor sensor divergence. */
    DEGRADED,
    /** Tracking lost; translation integration halted to prevent runaway dead-reckoning drift. */
    LOST,
    /** Attempting recovery against stored local keyframes and landmarks. */
    RELOCALIZING,
}

/**
 * Relative displacement and rotation evidence between consecutive camera frames.
 */
data class VisualRelativeMotion(
    val timestampNs: Long,
    /** Relative forward displacement in rover body frame (+X forward, meters or normalized units). */
    val deltaXM: Float,
    /** Relative lateral displacement in rover body frame (+Y left, meters or normalized units). */
    val deltaYM: Float,
    /** Relative visual rotation angle in radians (counter-clockwise positive). */
    val deltaYawRad: Float,
    val translationConfidence: Float,
    val rotationConfidence: Float,
    val overallConfidence: Float,
    val scaleState: MonocularScaleState = MonocularScaleState.SCALE_UNKNOWN,
    val isValid: Boolean = true,
) {
    companion object {
        val ZERO = VisualRelativeMotion(
            timestampNs = 0L,
            deltaXM = 0f,
            deltaYM = 0f,
            deltaYawRad = 0f,
            translationConfidence = 0f,
            rotationConfidence = 0f,
            overallConfidence = 0f,
            scaleState = MonocularScaleState.SCALE_UNKNOWN,
            isValid = false,
        )
    }
}

/**
 * Estimated 2D spatial pose of the rover relative to its startup origin (0, 0, 0°).
 */
data class LocalPose(
    val timestampNs: Long = 0L,
    /** Forward position in meters (or normalized units) relative to start origin. */
    val xM: Float = 0.0f,
    /** Lateral position in meters (or normalized units) relative to start origin (+Y left). */
    val yM: Float = 0.0f,
    /** Heading / Yaw angle in degrees relative to start origin (-180° to 180°, CCW positive). */
    val yawDeg: Float = 0.0f,
    /** Overall pose confidence score in range [0.0, 1.0]. */
    val confidence: Float = 1.0f,
    val visualConfidence: Float = 1.0f,
    val imuConfidence: Float = 1.0f,
    val scaleConfidence: Float = 0.0f,
    val scaleState: MonocularScaleState = MonocularScaleState.SCALE_UNKNOWN,
    val trackingState: TrackingQuality = TrackingQuality.INITIALIZING,
) {
    /** Formatted human-readable HUD string. */
    fun toSummaryString(): String {
        val xStr = String.format("%.2f", xM)
        val yStr = String.format("%.2f", yM)
        val yawStr = String.format("%.1f", yawDeg)
        val confPct = (confidence * 100f).roundToInt()
        return "X: ${xStr}m, Y: ${yStr}m, Yaw: ${yawStr}° [$trackingState, $confPct%]"
    }

    companion object {
        val ORIGIN = LocalPose(
            timestampNs = 0L,
            xM = 0.0f,
            yM = 0.0f,
            yawDeg = 0.0f,
            confidence = 1.0f,
            visualConfidence = 1.0f,
            imuConfidence = 1.0f,
            scaleConfidence = 0.0f,
            scaleState = MonocularScaleState.SCALE_UNKNOWN,
            trackingState = TrackingQuality.INITIALIZING,
        )
    }
}
