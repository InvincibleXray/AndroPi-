package com.minesafety.roboeye.fusion

import com.minesafety.roboeye.vision.MotionState
import com.minesafety.roboeye.vision.RecommendedCorridor
import com.minesafety.roboeye.vision.SectorRisk
import java.util.Locale

enum class FreeSpaceClearance {
    CLEAR_WITH_CONFIDENCE,
    BLOCKED,
    CONFLICTING,
    UNKNOWN,
}

data class SectorClearance(
    val sector: CorridorSector,
    val clearance: FreeSpaceClearance,
    val clearDistanceM: Float,
    val confidence: Float,
    val explanation: String,
)

data class FusedFreeSpace(
    val left: SectorClearance,
    val center: SectorClearance,
    val right: SectorClearance,
    val isClearAhead: Boolean,
    val minClearanceM: Float,
    val overallConfidence: Float,
    val recommendedCorridor: RecommendedCorridor = RecommendedCorridor.NONE_AVAILABLE,
)

/**
 * Geometric free-space estimator that projects physical sensors and visual evidence
 * against the rover's physical footprint and safety margins.
 *
 * Core rule: "UNKNOWN IS NOT SAFE". If a sector has stale or missing sensor evidence,
 * it is classified as UNKNOWN with confidence 0, never as CLEAR.
 */
class GeometricFreeSpaceEstimator(
    val footprint: RoverFootprint = RoverFootprint(),
    val eStopDistanceM: Float = 0.5f,
    val cautionDistanceM: Float = 1.5f,
    val maxCorridorRangeM: Float = 4.0f,
) {

    /**
     * Primary Phase 4 perception evaluation: estimates 3-corridor drivable free space
     * exclusively from monocular camera optical flow, regional TTC, and phone IMU tilt.
     */
    fun estimateFromVision(
        left: VisualFreeSpaceEvidence,
        center: VisualFreeSpaceEvidence,
        right: VisualFreeSpaceEvidence,
        motionState: MotionState = MotionState.UNKNOWN,
        cameraPitchDeg: Float = 0.0f,
    ): FusedFreeSpace {
        val leftClearance = evaluateVisualSector(left)
        val centerClearance = evaluateVisualSector(center)
        val rightClearance = evaluateVisualSector(right)

        val clearances = listOf(leftClearance, centerClearance, rightClearance)
        val minClearance = clearances.map { it.clearDistanceM }.minOrNull() ?: 0.0f
        val avgConfidence = clearances.map { it.confidence }.average().toFloat()

        // Forward passage rule: only clear ahead if CENTER is actively confirmed clear with sufficient margin
        val isClearAhead = centerClearance.clearance == FreeSpaceClearance.CLEAR_WITH_CONFIDENCE &&
                centerClearance.clearDistanceM >= cautionDistanceM &&
                centerClearance.confidence >= 0.4f &&
                motionState != MotionState.UNKNOWN &&
                motionState != MotionState.VISUAL_TRACKING_LOST

        // Determine recommended corridor
        val recommended = when {
            isClearAhead -> RecommendedCorridor.CENTER
            leftClearance.clearance == FreeSpaceClearance.CLEAR_WITH_CONFIDENCE &&
                leftClearance.clearDistanceM > centerClearance.clearDistanceM &&
                leftClearance.clearDistanceM >= cautionDistanceM -> RecommendedCorridor.LEFT_DETOUR
            rightClearance.clearance == FreeSpaceClearance.CLEAR_WITH_CONFIDENCE &&
                rightClearance.clearDistanceM > centerClearance.clearDistanceM &&
                rightClearance.clearDistanceM >= cautionDistanceM -> RecommendedCorridor.RIGHT_DETOUR
            leftClearance.clearDistanceM >= rightClearance.clearDistanceM && leftClearance.clearDistanceM >= cautionDistanceM -> RecommendedCorridor.LEFT_DETOUR
            rightClearance.clearDistanceM >= cautionDistanceM -> RecommendedCorridor.RIGHT_DETOUR
            else -> RecommendedCorridor.NONE_AVAILABLE
        }

        return FusedFreeSpace(
            left = leftClearance,
            center = centerClearance,
            right = rightClearance,
            isClearAhead = isClearAhead,
            minClearanceM = minClearance,
            overallConfidence = avgConfidence,
            recommendedCorridor = recommended,
        )
    }

    private fun evaluateVisualSector(ev: VisualFreeSpaceEvidence): SectorClearance {
        if (!ev.isFresh || ev.featureCount < 3 || ev.confidence < 0.15f) {
            // UNKNOWN IS NOT SAFE
            return SectorClearance(
                sector = ev.sector,
                clearance = FreeSpaceClearance.UNKNOWN,
                clearDistanceM = 0.0f,
                confidence = 0.0f,
                explanation = "Insufficient visual tracking features in ${ev.sector.name} sector (${ev.featureCount} features)",
            )
        }

        val isBlocked = ev.risk == SectorRisk.CRITICAL || ev.risk == SectorRisk.WARNING ||
                (ev.minTtcSec != null && ev.minTtcSec <= 2.5f)

        val clearanceState = if (isBlocked) FreeSpaceClearance.BLOCKED else FreeSpaceClearance.CLEAR_WITH_CONFIDENCE
        val dist = ev.clearanceDistanceM.coerceIn(0.2f, maxCorridorRangeM)

        val ttcStr = ev.minTtcSec?.let { String.format(Locale.US, "%.1fs", it) } ?: "low"
        val distStr = String.format(Locale.US, "%.1fm", dist)

        return SectorClearance(
            sector = ev.sector,
            clearance = clearanceState,
            clearDistanceM = dist,
            confidence = ev.confidence,
            explanation = if (isBlocked) "Visual hazard in ${ev.sector.name} (TTC: $ttcStr)"
                          else "Corridor traversable up to $distStr",
        )
    }

    /**
     * Multi-sensor cross-check evaluation (preserved for backward compatibility with sensor fusion test suite).
     */
    fun estimate(
        crossCheckLeft: SectorCrossCheckResult,
        crossCheckCenter: SectorCrossCheckResult,
        crossCheckRight: SectorCrossCheckResult,
    ): FusedFreeSpace {
        val left = evaluateSector(CorridorSector.LEFT, crossCheckLeft)
        val center = evaluateSector(CorridorSector.CENTER, crossCheckCenter)
        val right = evaluateSector(CorridorSector.RIGHT, crossCheckRight)

        val clearances = listOf(left, center, right)
        val minClearance = clearances.map { it.clearDistanceM }.minOrNull() ?: 0.0f
        val avgConfidence = clearances.map { it.confidence }.average().toFloat()

        // Forward passage rule: only clear ahead if CENTER is actively confirmed clear with sufficient margin
        val isClearAhead = center.clearance == FreeSpaceClearance.CLEAR_WITH_CONFIDENCE &&
                center.clearDistanceM >= cautionDistanceM &&
                center.confidence >= 0.4f

        val recommended = when {
            isClearAhead -> RecommendedCorridor.CENTER
            left.clearance == FreeSpaceClearance.CLEAR_WITH_CONFIDENCE && left.clearDistanceM >= cautionDistanceM -> RecommendedCorridor.LEFT_DETOUR
            right.clearance == FreeSpaceClearance.CLEAR_WITH_CONFIDENCE && right.clearDistanceM >= cautionDistanceM -> RecommendedCorridor.RIGHT_DETOUR
            else -> RecommendedCorridor.NONE_AVAILABLE
        }

        return FusedFreeSpace(
            left = left,
            center = center,
            right = right,
            isClearAhead = isClearAhead,
            minClearanceM = minClearance,
            overallConfidence = avgConfidence,
            recommendedCorridor = recommended,
        )
    }

    private fun evaluateSector(
        sector: CorridorSector,
        crossCheck: SectorCrossCheckResult,
    ): SectorClearance {
        return when (crossCheck.agreement) {
            CrossCheckAgreement.UNKNOWN -> {
                // UNKNOWN IS NOT SAFE: Never assume clear when sensor data is missing
                SectorClearance(
                    sector = sector,
                    clearance = FreeSpaceClearance.UNKNOWN,
                    clearDistanceM = 0.0f,
                    confidence = 0.0f,
                    explanation = "No sensor coverage in ${sector.name} sector",
                )
            }
            CrossCheckAgreement.CONFLICTING -> {
                val dist = crossCheck.physicalRangeM ?: cautionDistanceM
                SectorClearance(
                    sector = sector,
                    clearance = FreeSpaceClearance.CONFLICTING,
                    clearDistanceM = dist,
                    confidence = crossCheck.confidence,
                    explanation = "Sensors conflicting in ${sector.name} sector: ${crossCheck.description}",
                )
            }
            CrossCheckAgreement.CONSISTENT_OBSTACLE -> {
                val dist = crossCheck.physicalRangeM ?: eStopDistanceM
                SectorClearance(
                    sector = sector,
                    clearance = FreeSpaceClearance.BLOCKED,
                    clearDistanceM = dist,
                    confidence = crossCheck.confidence,
                    explanation = "Dual-sensor confirmed obstacle at ${dist}m",
                )
            }
            CrossCheckAgreement.CONSISTENT_CLEAR -> {
                val dist = crossCheck.physicalRangeM ?: maxCorridorRangeM
                SectorClearance(
                    sector = sector,
                    clearance = FreeSpaceClearance.CLEAR_WITH_CONFIDENCE,
                    clearDistanceM = dist,
                    confidence = crossCheck.confidence,
                    explanation = "Corridor clear up to ${dist}m",
                )
            }
            CrossCheckAgreement.RANGE_ONLY -> {
                val dist = crossCheck.physicalRangeM ?: maxCorridorRangeM
                val isBlocked = dist <= cautionDistanceM
                SectorClearance(
                    sector = sector,
                    clearance = if (isBlocked) FreeSpaceClearance.BLOCKED else FreeSpaceClearance.CLEAR_WITH_CONFIDENCE,
                    clearDistanceM = dist,
                    confidence = crossCheck.confidence,
                    explanation = crossCheck.description,
                )
            }
            CrossCheckAgreement.VISION_ONLY -> {
                val isBlocked = (crossCheck.visualTtcSec ?: 10.0f) <= 2.5f
                val approxDist = ((crossCheck.visualTtcSec ?: 5.0f) * 0.5f).coerceIn(0.2f, maxCorridorRangeM)
                SectorClearance(
                    sector = sector,
                    clearance = if (isBlocked) FreeSpaceClearance.BLOCKED else FreeSpaceClearance.CLEAR_WITH_CONFIDENCE,
                    clearDistanceM = approxDist,
                    confidence = crossCheck.confidence,
                    explanation = crossCheck.description,
                )
            }
        }
    }
}
