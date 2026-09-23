package com.minesafety.roboeye.vision

import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test

class RansacFlowFilterTest {

    private val filter = RansacFlowFilter(
        maxIterations = 18,
        inlierThresholdPx = 2.5f,
        minInliers = 6,
    )

    @Test
    fun `pure translation consensus identifies inliers and rejects wild outliers`() {
        val vectors = ArrayList<FlowVector>()
        // 20 inlier vectors moving at (dx = 5.0, dy = -2.0)
        for (i in 0 until 20) {
            val noise = (i % 3 - 1) * 0.3f
            vectors.add(
                FlowVector(
                    prevX = 100f + i * 10f,
                    prevY = 100f + i * 5f,
                    currX = 105f + i * 10f + noise,
                    currY = 98f + i * 5f + noise,
                    confidence = 0.9f,
                    isValid = true,
                )
            )
        }

        // 5 rogue outlier vectors pointing in wild directions
        vectors.add(FlowVector(50f, 50f, 85f, 20f, 0.8f, true))
        vectors.add(FlowVector(60f, 60f, 10f, 90f, 0.8f, true))
        vectors.add(FlowVector(70f, 70f, 70f, 120f, 0.8f, true))
        vectors.add(FlowVector(80f, 80f, 30f, 40f, 0.8f, true))
        vectors.add(FlowVector(90f, 90f, 150f, 150f, 0.8f, true))

        val result = filter.filter(vectors)

        // Must capture all 20 inliers
        assertEquals(20, result.inliers.size)
        assertEquals(5, result.outliersCount)
        assertTrue("Inlier ratio should be 20/25 = 0.80", result.inlierRatio >= 0.79f)
        assertEquals(5.0f, result.consensusDx, 0.5f)
        assertEquals(-2.0f, result.consensusDy, 0.5f)
        assertTrue("Iterations run should be <= 18", result.iterationsRun <= 18)
    }

    @Test
    fun `fewer vectors than minInliers bypasses RANSAC gracefully`() {
        val vectors = listOf(
            FlowVector(10f, 10f, 12f, 12f, 0.9f, true),
            FlowVector(20f, 20f, 22f, 22f, 0.9f, true),
            FlowVector(30f, 30f, 32f, 32f, 0.9f, true),
        )

        val result = filter.filter(vectors)
        assertEquals(3, result.inliers.size)
        assertEquals(0, result.outliersCount)
        assertEquals(0, result.iterationsRun)
        assertEquals(2.0f, result.consensusDx, 0.01f)
    }

    @Test
    fun `empty list returns empty result safely`() {
        val result = filter.filter(emptyList())
        assertEquals(0, result.inliers.size)
        assertEquals(0, result.outliersCount)
        assertEquals(0.0f, result.consensusDx, 0.001f)
        assertEquals(0.0f, result.inlierRatio, 0.001f)
    }
}
