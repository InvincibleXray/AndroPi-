package com.minesafety.roboeye.fusion

import com.minesafety.roboeye.vision.MotionState
import com.minesafety.roboeye.vision.RecommendedCorridor
import com.minesafety.roboeye.vision.SectorRisk
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Test

class VisualFreeSpaceEstimatorTest {

    private val estimator = GeometricFreeSpaceEstimator(
        cautionDistanceM = 1.5f,
        maxCorridorRangeM = 4.0f,
    )

    @Test
    fun `center clear corridor with sufficient margin reports clear ahead`() {
        val left = VisualFreeSpaceEvidence(
            sector = CorridorSector.LEFT,
            confidence = 0.8f,
            isFresh = true,
            clearanceDistanceM = 3.5f,
            risk = SectorRisk.SAFE,
            featureCount = 15,
            isDivergent = true,
            minTtcSec = 7.0f,
        )
        val center = VisualFreeSpaceEvidence(
            sector = CorridorSector.CENTER,
            confidence = 0.9f,
            isFresh = true,
            clearanceDistanceM = 3.8f,
            risk = SectorRisk.SAFE,
            featureCount = 25,
            isDivergent = true,
            minTtcSec = 8.0f,
        )
        val right = VisualFreeSpaceEvidence(
            sector = CorridorSector.RIGHT,
            confidence = 0.8f,
            isFresh = true,
            clearanceDistanceM = 3.5f,
            risk = SectorRisk.SAFE,
            featureCount = 18,
            isDivergent = true,
            minTtcSec = 7.0f,
        )

        val result = estimator.estimateFromVision(left, center, right, MotionState.FORWARD_TRANSLATION)

        assertTrue("Center corridor should be clear ahead", result.isClearAhead)
        assertEquals(FreeSpaceClearance.CLEAR_WITH_CONFIDENCE, result.center.clearance)
        assertEquals(RecommendedCorridor.CENTER, result.recommendedCorridor)
        assertTrue(result.center.clearDistanceM >= 3.0f)
    }

    @Test
    fun `center blocked with open left corridor recommends left detour`() {
        val left = VisualFreeSpaceEvidence(
            sector = CorridorSector.LEFT,
            confidence = 0.85f,
            isFresh = true,
            clearanceDistanceM = 3.2f,
            risk = SectorRisk.SAFE,
            featureCount = 18,
            isDivergent = true,
            minTtcSec = 6.4f,
        )
        val center = VisualFreeSpaceEvidence(
            sector = CorridorSector.CENTER,
            confidence = 0.9f,
            isFresh = true,
            clearanceDistanceM = 0.6f,
            risk = SectorRisk.CRITICAL,
            featureCount = 20,
            isDivergent = true,
            minTtcSec = 1.2f, // Hazard approaching in center!
        )
        val right = VisualFreeSpaceEvidence(
            sector = CorridorSector.RIGHT,
            confidence = 0.7f,
            isFresh = true,
            clearanceDistanceM = 0.8f,
            risk = SectorRisk.WARNING,
            featureCount = 12,
            isDivergent = true,
            minTtcSec = 1.6f,
        )

        val result = estimator.estimateFromVision(left, center, right, MotionState.FORWARD_TRANSLATION)

        assertFalse("Center must not be clear ahead", result.isClearAhead)
        assertEquals(FreeSpaceClearance.BLOCKED, result.center.clearance)
        assertEquals(RecommendedCorridor.LEFT_DETOUR, result.recommendedCorridor)
    }

    @Test
    fun `unknown is NOT safe rule - missing visual features forces UNKNOWN never CLEAR`() {
        val emptyCenter = VisualFreeSpaceEvidence(
            sector = CorridorSector.CENTER,
            confidence = 0.05f, // low confidence
            isFresh = true,
            clearanceDistanceM = 4.0f,
            risk = SectorRisk.SAFE,
            featureCount = 1, // too few features
            isDivergent = false,
        )
        val left = VisualFreeSpaceEvidence(
            CorridorSector.LEFT, 0.8f, true, 3.0f, SectorRisk.SAFE, 12, true
        )
        val right = VisualFreeSpaceEvidence(
            CorridorSector.RIGHT, 0.8f, true, 3.0f, SectorRisk.SAFE, 12, true
        )

        val result = estimator.estimateFromVision(left, emptyCenter, right, MotionState.FORWARD_TRANSLATION)

        assertFalse("Uninstrumented center must never be marked clear ahead", result.isClearAhead)
        assertEquals(FreeSpaceClearance.UNKNOWN, result.center.clearance)
        assertEquals(0.0f, result.center.confidence, 0.001f)
    }

    @Test
    fun `visual tracking lost forces isClearAhead false`() {
        val left = VisualFreeSpaceEvidence(CorridorSector.LEFT, 0.8f, true, 3.5f, SectorRisk.SAFE, 15, true)
        val center = VisualFreeSpaceEvidence(CorridorSector.CENTER, 0.8f, true, 3.5f, SectorRisk.SAFE, 15, true)
        val right = VisualFreeSpaceEvidence(CorridorSector.RIGHT, 0.8f, true, 3.5f, SectorRisk.SAFE, 15, true)

        val result = estimator.estimateFromVision(left, center, right, MotionState.VISUAL_TRACKING_LOST)

        assertFalse("Clear ahead must be false when visual tracking is lost", result.isClearAhead)
    }
}
