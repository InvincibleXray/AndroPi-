package com.minesafety.roboeye.vision

import org.junit.Assert.assertEquals
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertTrue
import org.junit.Test
import kotlin.math.abs

class RegionalTtcEstimatorTest {

    private val estimator = RegionalTtcEstimator(
        criticalTtcSec = 1.0f,
        warningTtcSec = 2.5f,
        cautionTtcSec = 5.0f,
        minSectorFeatures = 2,
    )

    @Test
    fun estimate_recoversTheoreticalTtc() {
        val foeX = 150f
        val foeY = 100f
        val dt = 0.1f // 10 FPS
        val targetTtc = 1.5f // expected TTC

        // Formula: TTC = (r / r_dot) * dt => r_dot = (r * dt) / TTC
        val r = 30f
        val rDot = (r * dt) / targetTtc // = (30 * 0.1) / 1.5 = 2.0 px/frame

        // Place points in CENTER sector (image width = 300, center is 100..200)
        val vectors = listOf(
            FlowVector(prevX = 150f, prevY = 130f, currX = 150f, currY = 130f + rDot, isValid = true),
            FlowVector(prevX = 150f, prevY = 70f, currX = 150f, currY = 70f - rDot, isValid = true),
            FlowVector(prevX = 170f, prevY = 100f, currX = 170f + rDot, currY = 100f, isValid = true),
        )

        val result = estimator.estimate(
            vectors = vectors,
            foeX = foeX,
            foeY = foeY,
            isDivergent = true,
            deltaSec = dt,
            imageWidth = 300,
            imageHeight = 200,
        )

        val centerTtc = result.center.medianTtcSec
        assertNotNull("Center sector TTC should not be null", centerTtc)
        assertTrue("Estimated TTC $centerTtc must be close to $targetTtc", abs(centerTtc!! - targetTtc) < 0.1f)
        assertEquals(SectorRisk.WARNING, result.center.risk)
    }

    @Test
    fun estimate_criticalTtcTriggered() {
        val foeX = 150f
        val foeY = 100f
        val dt = 0.1f
        val targetTtc = 0.5f // Critical (< 1.0s)
        val r1 = 90f // prevX at 60 -> distance from foeX (150) is 90
        val rDot1 = (r1 * dt) / targetTtc // = (90 * 0.1) / 0.5 = 18.0 px/frame

        val r2 = 100f // prevX at 50 -> distance from foeX (150) is 100
        val rDot2 = (r2 * dt) / targetTtc // = (100 * 0.1) / 0.5 = 20.0 px/frame

        // Points in LEFT sector (x < 100)
        val vectors = listOf(
            FlowVector(prevX = 60f, prevY = 100f, currX = 60f - rDot1, currY = 100f, isValid = true),
            FlowVector(prevX = 50f, prevY = 100f, currX = 50f - rDot2, currY = 100f, isValid = true),
        )

        val result = estimator.estimate(
            vectors = vectors,
            foeX = foeX,
            foeY = foeY,
            isDivergent = true,
            deltaSec = dt,
            imageWidth = 300,
            imageHeight = 200,
        )

        assertEquals(SectorRisk.CRITICAL, result.left.risk)
        assertEquals(SectorRisk.CRITICAL, result.overallRisk)
        assertEquals("LEFT", result.criticalSector)
    }
}
