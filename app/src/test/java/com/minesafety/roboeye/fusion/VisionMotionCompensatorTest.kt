package com.minesafety.roboeye.fusion

import com.minesafety.roboeye.core.ImuReading
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test

class VisionMotionCompensatorTest {

    private val compensator = VisionMotionCompensator(
        rotationThresholdDps = 15.0f,
        suppressionRangeDps = 30.0f,
    )

    @Test
    fun evaluate_forwardMotion_fullConfidencePreserved() {
        // Linear acceleration, negligible yaw rate (5°/s)
        val imu = ImuReading(axG = 0.1f, yawRateDps = 5.0f)
        val res = compensator.evaluate(imu)

        assertFalse("Low yaw rate should not be classified as rotational", res.isRotationalMotion)
        assertEquals(1.0f, res.confidenceMultiplier, 0.001f)
    }

    @Test
    fun evaluate_moderateTurning_dampsConfidence() {
        // Yaw rate = 30°/s (> 15°/s threshold, excess = 15°/s over 30°/s range -> factor 0.5)
        val imu = ImuReading(yawRateDps = 30.0f)
        val res = compensator.evaluate(imu)

        assertEquals(0.5f, res.confidenceMultiplier, 0.01f)
    }

    @Test
    fun evaluate_severeRotation_suppressesVisualTtc() {
        // High yaw rate = 50°/s (> 45°/s cutoff)
        val imu = ImuReading(yawRateDps = 50.0f)
        val res = compensator.evaluate(imu)

        assertTrue("High yaw rate must be flagged as rotational motion", res.isRotationalMotion)
        assertEquals(0.0f, res.confidenceMultiplier, 0.001f)

        // Compensate TTC: should suppress TTC alert to prevent false obstacle warnings during turn
        val (ttc, conf) = compensator.compensateTtc(rawTtcSec = 1.0f, rawConfidence = 0.9f, imu = imu)
        assertNull("Severe turn must suppress false visual TTC alert", ttc)
        assertEquals(0.0f, conf, 0.001f)
    }

    @Test
    fun evaluate_nullImu_preservesBaseline() {
        val res = compensator.evaluate(null)
        assertFalse(res.isRotationalMotion)
        assertEquals(1.0f, res.confidenceMultiplier, 0.001f)

        val (ttc, conf) = compensator.compensateTtc(2.0f, 0.8f, null)
        assertNotNull(ttc)
        assertEquals(0.8f, conf, 0.001f)
    }
}
