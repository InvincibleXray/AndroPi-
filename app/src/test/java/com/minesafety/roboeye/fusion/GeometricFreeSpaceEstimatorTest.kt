package com.minesafety.roboeye.fusion

import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Test

class GeometricFreeSpaceEstimatorTest {

    private val estimator = GeometricFreeSpaceEstimator(
        eStopDistanceM = 0.5f,
        cautionDistanceM = 1.5f,
        maxCorridorRangeM = 4.0f,
    )

    private fun makeResult(sector: CorridorSector, agreement: CrossCheckAgreement, dist: Float?, conf: Float = 0.8f): SectorCrossCheckResult {
        return SectorCrossCheckResult(
            sector = sector,
            agreement = agreement,
            physicalRangeM = dist,
            visualTtcSec = null,
            confidence = conf,
            description = "Test result",
        )
    }

    @Test
    fun estimate_allClear_confirmsClearAhead() {
        val left = makeResult(CorridorSector.LEFT, CrossCheckAgreement.CONSISTENT_CLEAR, 3.5f)
        val center = makeResult(CorridorSector.CENTER, CrossCheckAgreement.CONSISTENT_CLEAR, 3.5f)
        val right = makeResult(CorridorSector.RIGHT, CrossCheckAgreement.CONSISTENT_CLEAR, 3.5f)

        val freeSpace = estimator.estimate(left, center, right)

        assertTrue("Center should be clear ahead", freeSpace.isClearAhead)
        assertEquals(FreeSpaceClearance.CLEAR_WITH_CONFIDENCE, freeSpace.center.clearance)
        assertEquals(3.5f, freeSpace.center.clearDistanceM, 0.001f)
    }

    @Test
    fun estimate_centerBlocked_reportsNotClearAhead() {
        val left = makeResult(CorridorSector.LEFT, CrossCheckAgreement.CONSISTENT_CLEAR, 3.5f)
        val center = makeResult(CorridorSector.CENTER, CrossCheckAgreement.CONSISTENT_OBSTACLE, 0.8f)
        val right = makeResult(CorridorSector.RIGHT, CrossCheckAgreement.CONSISTENT_CLEAR, 3.5f)

        val freeSpace = estimator.estimate(left, center, right)

        assertFalse("Center obstacle must block isClearAhead", freeSpace.isClearAhead)
        assertEquals(FreeSpaceClearance.BLOCKED, freeSpace.center.clearance)
        assertEquals(0.8f, freeSpace.center.clearDistanceM, 0.001f)
    }

    @Test
    fun estimate_unknownIsNotSafe() {
        // Missing sensor coverage in center
        val left = makeResult(CorridorSector.LEFT, CrossCheckAgreement.CONSISTENT_CLEAR, 3.5f)
        val center = makeResult(CorridorSector.CENTER, CrossCheckAgreement.UNKNOWN, null, 0.0f)
        val right = makeResult(CorridorSector.RIGHT, CrossCheckAgreement.CONSISTENT_CLEAR, 3.5f)

        val freeSpace = estimator.estimate(left, center, right)

        // UNKNOWN IS NOT SAFE rule
        assertFalse("Unknown center sector must NEVER be marked clear ahead", freeSpace.isClearAhead)
        assertEquals(FreeSpaceClearance.UNKNOWN, freeSpace.center.clearance)
        assertEquals(0.0f, freeSpace.center.clearDistanceM, 0.001f)
        assertEquals(0.0f, freeSpace.center.confidence, 0.001f)
    }
}
