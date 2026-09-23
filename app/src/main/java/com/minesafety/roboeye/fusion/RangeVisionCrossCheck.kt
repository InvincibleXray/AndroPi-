package com.minesafety.roboeye.fusion

enum class CrossCheckAgreement {
    CONSISTENT_OBSTACLE,
    CONSISTENT_CLEAR,
    RANGE_ONLY,
    VISION_ONLY,
    CONFLICTING,
    UNKNOWN,
}

data class SectorCrossCheckResult(
    val sector: CorridorSector,
    val agreement: CrossCheckAgreement,
    val physicalRangeM: Float?,
    val visualTtcSec: Float?,
    val confidence: Float,
    val description: String,
)

/**
 * Cross-checks monocular visual expansion (TTC) against physical range telemetry (ultrasonic/ToF).
 *
 * Never fabricates consensus: explicitly preserves sensor disagreement for safety evaluation.
 */
class RangeVisionCrossCheck(
    val rangeObstacleThresholdM: Float = 1.5f,
    val visualObstacleThresholdSec: Float = 2.5f,
    val rangeImmediateHazardM: Float = 0.4f,
    val visualImmediateHazardSec: Float = 1.0f,
) {

    fun crossCheckSector(
        sector: CorridorSector,
        range: ValidatedRange?,
        visualTtcSec: Float?,
        visualConfidence: Float,
    ): SectorCrossCheckResult {
        val hasRange = range != null && range.isValid && range.distanceM != null
        val hasVision = visualTtcSec != null && visualTtcSec > 0.0f && visualConfidence > 0.2f

        if (!hasRange && !hasVision) {
            return SectorCrossCheckResult(
                sector = sector,
                agreement = CrossCheckAgreement.UNKNOWN,
                physicalRangeM = null,
                visualTtcSec = null,
                confidence = 0.0f,
                description = "No valid range or vision data available in ${sector.name} sector",
            )
        }

        val validRangeDist = range?.distanceM
        if (hasRange && !hasVision) {
            val dist = validRangeDist ?: 0.0f
            val isObs = dist <= rangeObstacleThresholdM
            return SectorCrossCheckResult(
                sector = sector,
                agreement = CrossCheckAgreement.RANGE_ONLY,
                physicalRangeM = dist,
                visualTtcSec = null,
                confidence = range!!.confidence * 0.8f,
                description = if (isObs) "Physical range obstacle detected at ${dist}m (vision unconfirmed)"
                else "Physical range clear at ${dist}m (vision unconfirmed)",
            )
        }

        if (!hasRange && hasVision) {
            val ttc = visualTtcSec ?: 0.0f
            val isObs = ttc <= visualObstacleThresholdSec
            return SectorCrossCheckResult(
                sector = sector,
                agreement = CrossCheckAgreement.VISION_ONLY,
                physicalRangeM = null,
                visualTtcSec = ttc,
                confidence = visualConfidence * 0.7f,
                description = if (isObs) "Visual collision risk detected (TTC=${String.format(java.util.Locale.US, "%.1f", ttc)}s, physical range unconfirmed)"
                else "Visual field clear (physical range unconfirmed)",
            )
        }

        // Both sources are active: cross-check them
        val dist = range?.distanceM ?: return SectorCrossCheckResult(
            sector = sector,
            agreement = CrossCheckAgreement.UNKNOWN,
            physicalRangeM = null,
            visualTtcSec = visualTtcSec,
            confidence = 0.0f,
            description = "Missing distance value",
        )
        val ttc = visualTtcSec ?: return SectorCrossCheckResult(
            sector = sector,
            agreement = CrossCheckAgreement.UNKNOWN,
            physicalRangeM = dist,
            visualTtcSec = null,
            confidence = 0.0f,
            description = "Missing visual TTC",
        )

        val rangeHazard = dist <= rangeObstacleThresholdM
        val visionHazard = ttc <= visualObstacleThresholdSec

        val rangeCritical = dist <= rangeImmediateHazardM
        val visionCritical = ttc <= visualImmediateHazardSec

        // Case 1: Severe conflict (one says emergency collision, other says wide open)
        if ((rangeCritical && !visionHazard) || (visionCritical && !rangeHazard)) {
            return SectorCrossCheckResult(
                sector = sector,
                agreement = CrossCheckAgreement.CONFLICTING,
                physicalRangeM = dist,
                visualTtcSec = ttc,
                confidence = 0.4f,
                description = "Conflict: range=${dist}m vs visual TTC=${ttc}s in ${sector.name} sector",
            )
        }

        // Case 2: Consistent obstacle confirmation
        if (rangeHazard && visionHazard) {
            val fusedConf = (range.confidence * 0.5f + visualConfidence * 0.5f + 0.1f).coerceIn(0.0f, 1.0f)
            return SectorCrossCheckResult(
                sector = sector,
                agreement = CrossCheckAgreement.CONSISTENT_OBSTACLE,
                physicalRangeM = dist,
                visualTtcSec = ttc,
                confidence = fusedConf,
                description = "Dual-sensor confirmed obstacle at ${dist}m, TTC=${ttc}s",
            )
        }

        // Case 3: Consistent clear corridor
        if (!rangeHazard && !visionHazard) {
            val fusedConf = (range.confidence * 0.5f + visualConfidence * 0.5f).coerceIn(0.0f, 1.0f)
            return SectorCrossCheckResult(
                sector = sector,
                agreement = CrossCheckAgreement.CONSISTENT_CLEAR,
                physicalRangeM = dist,
                visualTtcSec = ttc,
                confidence = fusedConf,
                description = "Dual-sensor confirmed clear corridor in ${sector.name} sector",
            )
        }

        // Case 4: Moderate asymmetry (one sensor sees approaching target before the other reaches threshold)
        return if (rangeHazard) {
            SectorCrossCheckResult(
                sector = sector,
                agreement = CrossCheckAgreement.RANGE_ONLY,
                physicalRangeM = dist,
                visualTtcSec = ttc,
                confidence = range.confidence * 0.75f,
                description = "Physical range alert (${dist}m) with moderate visual TTC (${ttc}s)",
            )
        } else {
            SectorCrossCheckResult(
                sector = sector,
                agreement = CrossCheckAgreement.VISION_ONLY,
                physicalRangeM = dist,
                visualTtcSec = ttc,
                confidence = visualConfidence * 0.7f,
                description = "Visual expansion alert (${ttc}s) with clear physical range (${dist}m)",
            )
        }
    }
}
