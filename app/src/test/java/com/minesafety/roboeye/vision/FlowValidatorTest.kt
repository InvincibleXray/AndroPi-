package com.minesafety.roboeye.vision

import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test

class FlowValidatorTest {

    private val validator = FlowValidator(maxJumpPx = 45.0f, minNoiseThresholdPx = 0.3f, boundaryMarginPx = 5.0f)

    @Test
    fun validate_filtersOutOfBoundsVectors() {
        val vectors = listOf(
            FlowVector(prevX = 10f, prevY = 10f, currX = -2f, currY = 10f, isValid = true), // OOB
            FlowVector(prevX = 10f, prevY = 10f, currX = 15f, currY = 15f, isValid = true), // In bounds
        )
        val res = validator.validate(vectors, imageWidth = 100, imageHeight = 100)
        assertEquals(1, res.validVectors.size)
        assertEquals(1, res.rejectedCount)
    }

    @Test
    fun validate_rejectsExcessiveJumps() {
        val vectors = listOf(
            FlowVector(prevX = 50f, prevY = 50f, currX = 110f, currY = 50f, isValid = true), // jump = 60 > 45
            FlowVector(prevX = 50f, prevY = 50f, currX = 55f, currY = 50f, isValid = true), // jump = 5 <= 45
        )
        val res = validator.validate(vectors, imageWidth = 200, imageHeight = 200)
        assertEquals(1, res.validVectors.size)
        assertEquals(1, res.rejectedCount)
    }

    @Test
    fun validate_handlesSubpixelNoise() {
        val vectors = listOf(
            FlowVector(prevX = 50f, prevY = 50f, currX = 50.1f, currY = 50.1f, isValid = true), // len ~ 0.14 < 0.3
        )
        val res = validator.validate(vectors, imageWidth = 200, imageHeight = 200)
        assertEquals(1, res.validVectors.size)
        assertEquals(1, res.stationaryCount)
        assertEquals(0.0f, res.validVectors[0].length, 0.001f)
    }

    @Test
    fun validate_filtersStatisticalOutliersViaIqr() {
        // 9 vectors with small displacement (~2px) and 1 extreme outlier (40px)
        val normalVectors = (1..9).map { i ->
            FlowVector(prevX = 50f + i * 5, prevY = 50f, currX = 52f + i * 5, currY = 50f, isValid = true)
        }
        val outlier = FlowVector(prevX = 100f, prevY = 50f, currX = 140f, currY = 50f, isValid = true) // 40px

        val res = validator.validate(normalVectors + outlier, imageWidth = 300, imageHeight = 300)
        assertTrue("Outlier should be rejected by IQR filter", res.rejectedCount >= 1)
        assertTrue("Normal vectors should survive", res.validVectors.size >= 9)
    }
}
