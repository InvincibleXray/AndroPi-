package com.minesafety.roboeye.fusion

import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test

class SensorValidatorTest {

    private val validator = SensorValidator(
        maxRangeFreshnessMs = 500L,
        maxImuFreshnessMs = 250L,
        maxJumpM = 2.5f,
        maxJumpIntervalMs = 100L,
    )

    @Test
    fun validateRange_validDistanceAccepted() {
        val now = 1000L
        val res = validator.validateRange("FRONT", 1.5f, readingTimestampMs = 900L, nowMs = now)

        assertEquals(ReadingStatus.VALID, res.status)
        assertEquals(1.5f, res.distanceM!!, 0.001f)
        assertTrue("Confidence should be high for fresh reading", res.confidence >= 0.8f)
    }

    @Test
    fun validateRange_nullAndOutOfRangeRejected() {
        val now = 1000L

        // Null reading
        val resNull = validator.validateRange("FRONT", null, readingTimestampMs = now, nowMs = now)
        assertEquals(ReadingStatus.UNAVAILABLE, resNull.status)
        assertNull(resNull.distanceM)

        // Below min range (0.02m)
        val resTooClose = validator.validateRange("FRONT", 0.01f, readingTimestampMs = now, nowMs = now)
        assertEquals(ReadingStatus.INVALID, resTooClose.status)

        // Above max range (4.5m)
        val resTooFar = validator.validateRange("FRONT", 5.0f, readingTimestampMs = now, nowMs = now)
        assertEquals(ReadingStatus.INVALID, resTooFar.status)

        // Non-finite reading
        val resNaN = validator.validateRange("FRONT", Float.NaN, readingTimestampMs = now, nowMs = now)
        assertEquals(ReadingStatus.INVALID, resNaN.status)
    }

    @Test
    fun validateRange_staleReadingIdentified() {
        val now = 2000L
        val readingTime = 1200L // 800ms old (> 500ms threshold)
        val res = validator.validateRange("FRONT", 1.5f, readingTimestampMs = readingTime, nowMs = now)

        assertEquals(ReadingStatus.STALE, res.status)
        assertEquals(0.0f, res.confidence, 0.001f)
    }

    @Test
    fun validateRange_jumpDiscontinuityRejected() {
        val t1 = 1000L
        validator.validateRange("FRONT", 1.0f, readingTimestampMs = t1, nowMs = t1)

        // 30ms later, jump from 1.0m to 4.0m (> 2.5m jump in <= 100ms)
        val t2 = 1030L
        val resJump = validator.validateRange("FRONT", 4.0f, readingTimestampMs = t2, nowMs = t2)

        assertEquals(ReadingStatus.INVALID, resJump.status)
    }

    @Test
    fun freshnessHelpers_correctlyIdentifyTimeLimits() {
        val now = 1000L
        assertTrue(validator.isImuFresh(800L, now)) // 200ms age <= 250ms
        assertFalse(validator.isImuFresh(700L, now)) // 300ms age > 250ms

        assertTrue(validator.isVisionFresh(600L, now)) // 400ms age <= 500ms
        assertFalse(validator.isVisionFresh(400L, now)) // 600ms age > 500ms
    }
}
