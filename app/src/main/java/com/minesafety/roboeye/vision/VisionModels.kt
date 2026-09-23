package com.minesafety.roboeye.vision

/**
 * Risk classification level for spatial sectors ahead of the rover.
 */
enum class SectorRisk {
    SAFE,
    CAUTION,
    WARNING,
    CRITICAL
}

/**
 * Time-To-Collision estimation for a single spatial sector.
 */
data class SectorTtc(
    val sectorName: String, // "LEFT", "CENTER", "RIGHT"
    val medianTtcSec: Float?,
    val minTtcSec: Float?,
    val risk: SectorRisk,
    val featureCount: Int,
)

/**
 * Comprehensive 3-sector Time-To-Collision assessment.
 */
data class RegionalTtcResult(
    val left: SectorTtc,
    val center: SectorTtc,
    val right: SectorTtc,
    val criticalSector: String? = null,
    val minTtcSec: Float? = null,
    val overallRisk: SectorRisk = SectorRisk.SAFE,
)

/**
 * Discrete deterministic motion state derived from visual-inertial sensor fusion.
 */
enum class MotionState {
    STATIONARY,
    FORWARD_TRANSLATION,
    BACKWARD_TRANSLATION,
    TURNING_LEFT,
    TURNING_RIGHT,
    COMBINED_MOTION,
    VISUAL_TRACKING_LOST,
    IMU_ONLY,
    UNKNOWN
}

/**
 * Cross-sensor agreement state between camera optical flow and phone IMU.
 */
enum class VisualImuConsistency {
    CONSISTENT,
    PARTIALLY_CONSISTENT,
    CONFLICTING,
    INSUFFICIENT_DATA
}

/**
 * Operational trustworthiness level of camera and scene geometry.
 */
enum class GeometryTrustLevel {
    TRUSTED,
    DEGRADED,
    UNTRUSTED
}

/**
 * Recommended traversable ground corridor for the rover.
 */
enum class RecommendedCorridor {
    CENTER,
    LEFT_DETOUR,
    RIGHT_DETOUR,
    NONE_AVAILABLE
}

/**
 * Summary of the local 2D visual occupancy grid ahead of the rover.
 */
data class VisualOccupancySummary(
    val occupiedCellsCount: Int,
    val freeCellsCount: Int,
    val uncertainCellsCount: Int,
    val nearestHazardDistanceM: Float?,
    val centerClearanceM: Float,
)

/**
 * Full output of the lightweight geometric vision and visual-inertial motion pipeline pass.
 */
data class GeometricVisionResult(
    val detectedFeaturesCount: Int,
    val trackedFeaturesCount: Int,
    val validFlowCount: Int,
    val foeX: Float,
    val foeY: Float,
    val isDivergent: Boolean,
    val foeConfidence: Float,
    val ttcResult: RegionalTtcResult,
    val processingTimeMs: Long,
    val motionState: MotionState = MotionState.UNKNOWN,
    val consistency: VisualImuConsistency = VisualImuConsistency.INSUFFICIENT_DATA,
    val motionConfidence: Float = 0.0f,
    val inlierCount: Int = 0,
    val rotationalDps: Float = 0.0f,
    val isImuAvailable: Boolean = false,
    val ransacIterations: Int = 0,
    val freeSpace: com.minesafety.roboeye.fusion.FusedFreeSpace? = null,
    val occupancySummary: VisualOccupancySummary? = null,
    val geometryTrust: GeometryTrustLevel = GeometryTrustLevel.UNTRUSTED,
    val recommendedCorridor: RecommendedCorridor = RecommendedCorridor.NONE_AVAILABLE,
    val localPose: com.minesafety.roboeye.localization.LocalPose? = null,
    val mapState: com.minesafety.roboeye.mapping.LocalMapState? = null,
)
