package com.minesafety.roboeye.fusion

import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test

class RangeVisionCrossCheckTest {

    private val crossCheck = RangeVisionCrossCheck(
        rangeObstacleThresholdM = 1.5f,
        visualObstacleThresholdSec = 2.5f,
        rangeImmediateHazardM = 0.4f,
        visualImmediateHazardSec = 1.0f,
    )

    @Test
    fun crossCheck_consistentObstacle_bothAgreeOnHazard() {
        val range = ValidatedRange(distanceM = 1.0f, status = ReadingStatus.VALID, confidence = 0.9f, ageMs = 50L)
        val res = crossCheck.crossCheckSector(
            sector = CorridorSector.CENTER,
            range = range,
            visualTtcSec = 1.8f,
            visualConfidence = 0.85f,
        )

        assertEquals(CrossCheckAgreement.CONSISTENT_OBSTACLE, res.agreement)
        assertTrue("Dual confirmation must have high confidence", res.confidence >= 0.9f)
    }

    @Test
    fun crossCheck_consistentClear_bothAgreeOnClearCorridor() {
        val range = ValidatedRange(distanceM = 3.5f, status = ReadingStatus.VALID, confidence = 0.8f, ageMs = 50L)
        val res = crossCheck.crossCheckSector(
            sector = CorridorSector.CENTER,
            range = range,
            visualTtcSec = 8.0f,
            visualConfidence = 0.8f,
        )

        assertEquals(CrossCheckAgreement.CONSISTENT_CLEAR, res.agreement)
    }

    @Test
    fun crossCheck_rangeOnly_physicalObstacleWithoutVisual() {
        val range = ValidatedRange(distanceM = 0.8f, status = ReadingStatus.VALID, confidence = 0.9f, ageMs = 50L)
        val res = crossCheck.crossCheckSector(
            sector = CorridorSector.LEFT,
            range = range,
            visualTtcSec = null,
            visualConfidence = 0.0f,
        )

        assertEquals(CrossCheckAgreement.RANGE_ONLY, res.agreement)
    }

    @Test
    fun crossCheck_visionOnly_visualAlertWithoutPhysicalRange() {
        val res = crossCheck.crossCheckSector(
            sector = CorridorSector.RIGHT,
            range = null,
            visualTtcSec = 1.5f,
            visualConfidence = 0.8f,
        )

        assertEquals(CrossCheckAgreement.VISION_ONLY, res.agreement)
    }

    @Test
    fun crossCheck_conflicting_immediateHazardVsClear() {
        // Immediate physical hazard (0.3m <= 0.4m) but vision reports no hazard (6.0s > 2.5s)
        val range = ValidatedRange(distanceM = 0.3f, status = ReadingStatus.VALID, confidence = 0.9f, ageMs = 50L)
        val res = crossCheck.crossCheckSector(
            sector = CorridorSector.CENTER,
            range = range,
            visualTtcSec = 6.0f,
            visualConfidence = 0.8f,
        )

        assertEquals(CrossCheckAgreement.CONFLICTING, res.agreement)
    }

    @Test
    fun crossCheck_unknown_whenNoData() {
        val res = crossCheck.crossCheckSector(
            sector = CorridorSector.CENTER,
            range = null,
            visualTtcSec = null,
            visualConfidence = 0.0f,
        )

        assertEquals(CrossCheckAgreement.UNKNOWN, res.agreement)
        assertEquals(0.0f, res.confidence, 0.001f)
    }
}
