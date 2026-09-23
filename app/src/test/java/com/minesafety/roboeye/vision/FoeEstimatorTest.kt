package com.minesafety.roboeye.vision

import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Test
import kotlin.math.abs

class FoeEstimatorTest {

    private val estimator = RobustFoeEstimator(minMovingVectors = 4, minDivergenceRatio = 0.65f)

    @Test
    fun estimate_recoversSyntheticExpansionCenter() {
        val trueFoeX = 160f
        val trueFoeY = 120f

        // Generate synthetic radial expansion vectors from (160, 120)
        val vectors = ArrayList<FlowVector>()
        val angles = listOf(0.0, 0.78, 1.57, 2.35, 3.14, 3.92, 4.71, 5.49)
        for (theta in angles) {
            val radius = 50f
            val px = trueFoeX + radius * kotlin.math.cos(theta).toFloat()
            val py = trueFoeY + radius * kotlin.math.sin(theta).toFloat()
            val speed = 3.0f
            val cx = px + speed * kotlin.math.cos(theta).toFloat()
            val cy = py + speed * kotlin.math.sin(theta).toFloat()
            vectors.add(FlowVector(px, py, cx, cy, confidence = 1.0f, isValid = true))
        }

        val result = estimator.estimate(vectors, imageWidth = 320, imageHeight = 240)

        assertTrue("Flow field must be recognized as divergent", result.isDivergent)
        assertTrue("Estimated FOE X must be close to $trueFoeX, was ${result.x}", abs(result.x - trueFoeX) < 5.0f)
        assertTrue("Estimated FOE Y must be close to $trueFoeY, was ${result.y}", abs(result.y - trueFoeY) < 5.0f)
        assertTrue("Confidence should be high (> 0.5)", result.confidence > 0.5f)
    }

    @Test
    fun estimate_parallelFlow_rejectsDivergence() {
        // Pure lateral translation (e.g. pan): all vectors move right (+5, 0)
        val vectors = (1..10).map { i ->
            val px = 50f + i * 15
            val py = 80f
            FlowVector(prevX = px, prevY = py, currX = px + 5f, currY = py, confidence = 1.0f, isValid = true)
        }

        val result = estimator.estimate(vectors, imageWidth = 320, imageHeight = 240)
        assertFalse("Parallel flow should not be divergent", result.isDivergent)
        assertEquals(0.0f, result.confidence, 0.001f)
    }
}
