package com.minesafety.roboeye.vision

import kotlin.math.hypot

/**
 * Regional Time-To-Collision (TTC) estimator based on first-principles radial optical flow expansion.
 *
 * Divides the camera field of view into three spatial sectors:
 * - LEFT (0.00 to 0.33 of image width)
 * - CENTER (0.33 to 0.67 of image width)
 * - RIGHT (0.67 to 1.00 of image width)
 *
 * For each expanding feature:
 *   r = distance from Focus of Expansion (FOE)
 *   r_dot = radial velocity component
 *   TTC = r / r_dot * delta_t
 */
class RegionalTtcEstimator(
    val criticalTtcSec: Float = 1.0f,
    val warningTtcSec: Float = 2.5f,
    val cautionTtcSec: Float = 5.0f,
    val minSectorFeatures: Int = 3,
    val minRadiusPx: Float = 8.0f,
    val minRadialVelocityPx: Float = 0.25f,
) {

    fun estimate(
        vectors: List<FlowVector>,
        foeX: Float,
        foeY: Float,
        isDivergent: Boolean,
        deltaSec: Float,
        imageWidth: Int,
        imageHeight: Int
    ): RegionalTtcResult {
        // If flow field is not divergent (e.g. rotating, stationary, or parallel), TTC cannot be reliably computed
        if (!isDivergent || deltaSec <= 0.0f) {
            return RegionalTtcResult(
                left = SectorTtc("LEFT", null, null, SectorRisk.SAFE, 0),
                center = SectorTtc("CENTER", null, null, SectorRisk.SAFE, 0),
                right = SectorTtc("RIGHT", null, null, SectorRisk.SAFE, 0),
                criticalSector = null,
                minTtcSec = null,
                overallRisk = SectorRisk.SAFE,
            )
        }

        val leftTtcList = ArrayList<Float>()
        val centerTtcList = ArrayList<Float>()
        val rightTtcList = ArrayList<Float>()

        val leftBound = imageWidth * 0.33f
        val rightBound = imageWidth * 0.67f

        for (v in vectors) {
            if (!v.isValid) continue

            val rx = v.prevX - foeX
            val ry = v.prevY - foeY
            val r = hypot(rx, ry)
            if (r < minRadiusPx) continue

            // Radial velocity: dot product of displacement vector with unit radial vector
            val rDot = (v.dx * rx + v.dy * ry) / r
            if (rDot < minRadialVelocityPx) continue // not expanding or moving inward

            // TTC calculation in seconds
            val ttc = (r / rDot) * deltaSec
            if (ttc <= 0.0f || ttc > 60.0f) continue // clamp unrealistic values

            when {
                v.prevX < leftBound -> leftTtcList.add(ttc)
                v.prevX > rightBound -> rightTtcList.add(ttc)
                else -> centerTtcList.add(ttc)
            }
        }

        val leftSector = assessSector("LEFT", leftTtcList)
        val centerSector = assessSector("CENTER", centerTtcList)
        val rightSector = assessSector("RIGHT", rightTtcList)

        // Find critical sector and overall risk
        val sectors = listOf(leftSector, centerSector, rightSector)
        val highestRisk = sectors.maxByOrNull { it.risk.ordinal }?.risk ?: SectorRisk.SAFE

        val minTtc = sectors.mapNotNull { it.minTtcSec }.minOrNull()
        val criticalSec = sectors.firstOrNull { it.risk == SectorRisk.CRITICAL }?.sectorName
            ?: sectors.firstOrNull { it.risk == SectorRisk.WARNING }?.sectorName

        return RegionalTtcResult(
            left = leftSector,
            center = centerSector,
            right = rightSector,
            criticalSector = criticalSec,
            minTtcSec = minTtc,
            overallRisk = highestRisk,
        )
    }

    private fun assessSector(name: String, ttcValues: List<Float>): SectorTtc {
        if (ttcValues.size < minSectorFeatures) {
            return SectorTtc(name, null, null, SectorRisk.SAFE, ttcValues.size)
        }

        val sorted = ttcValues.sorted()
        val median = sorted[sorted.size / 2]
        val minTtc = sorted.first()

        val risk = when {
            median < criticalTtcSec || minTtc < (criticalTtcSec * 0.6f) -> SectorRisk.CRITICAL
            median < warningTtcSec -> SectorRisk.WARNING
            median < cautionTtcSec -> SectorRisk.CAUTION
            else -> SectorRisk.SAFE
        }

        return SectorTtc(name, median, minTtc, risk, ttcValues.size)
    }
}
