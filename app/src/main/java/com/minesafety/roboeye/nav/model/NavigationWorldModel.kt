package com.minesafety.roboeye.nav.model

import com.minesafety.roboeye.localization.LocalPose
import com.minesafety.roboeye.localization.TrackingQuality
import com.minesafety.roboeye.mapping.LocalMapState
import com.minesafety.roboeye.mapping.LocalSpatialMap
import com.minesafety.roboeye.vision.GeometryTrustLevel
import com.minesafety.roboeye.vision.MotionState
import com.minesafety.roboeye.vision.RecommendedCorridor
import com.minesafety.roboeye.vision.RegionalTtcResult

/**
 * Aggregated navigation world model capturing spatial, perceptual, and kinematic state.
 *
 * Planners and motion controllers consume this interpreted world model rather than
 * raw camera frames or raw IMU streams.
 */
data class NavigationWorldModel(
    val pose: LocalPose = LocalPose.ORIGIN,
    val spatialMap: LocalSpatialMap,
    val mapSummary: LocalMapState = LocalMapState(),
    val geometryTrust: GeometryTrustLevel = GeometryTrustLevel.UNTRUSTED,
    val motionState: MotionState = MotionState.UNKNOWN,
    val recommendedCorridor: RecommendedCorridor = RecommendedCorridor.CENTER,
    val ttcResult: RegionalTtcResult? = null,
    val visualConfidence: Float = 0.0f,
    val timestampMs: Long = System.currentTimeMillis(),
    val roverWidthM: Float = 0.35f,
    val roverLengthM: Float = 0.45f,
    val safetyMarginM: Float = 0.15f,
    val semanticObjects: List<WorldObject> = emptyList(),
) {
    /** Combined collision radius for footprint inflation. */
    val inflationRadiusM: Float
        get() = (maxOf(roverWidthM, roverLengthM) / 2.0f) + safetyMarginM

    /** True if localization is tracking normally or safely degraded with sufficient confidence. */
    val isLocalizationValid: Boolean
        get() = pose.trackingState == TrackingQuality.TRACKING ||
                (pose.trackingState == TrackingQuality.DEGRADED && pose.confidence >= 0.40f)

    /** Checks whether the world model observations are too stale to navigate safely. */
    fun isStale(nowMs: Long = System.currentTimeMillis(), maxAgeMs: Long = 1000L): Boolean {
        return (nowMs - timestampMs) > maxAgeMs
    }

    /** True if the rover can safely proceed with autonomous navigation. */
    val isNavigable: Boolean
        get() = isLocalizationValid &&
                geometryTrust != GeometryTrustLevel.UNTRUSTED &&
                !isStale()
}
