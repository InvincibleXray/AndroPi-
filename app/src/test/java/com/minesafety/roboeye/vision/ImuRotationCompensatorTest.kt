package com.minesafety.roboeye.vision

import com.minesafety.roboeye.core.ImuReading
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Test

class ImuRotationCompensatorTest {

    private val compensator = ImuRotationCompensator(
        nominalFocalLengthPx = 550.0f,
        rotationThresholdDps = 15.0f,
        severeRotationThresholdDps = 45.0f,
    )

    @Test
    fun `stationary IMU preserves optical flow vectors and confidence`() {
        val vectors = listOf(
            FlowVector(100f, 100f, 105f, 102f, 0.9f, true),
            FlowVector(200f, 200f, 205f, 202f, 0.9f, true),
        )
        val imu = ImuReading(
            gyroX = 0.0f,
            gyroY = 0.0f,
            gyroZ = 0.0f,
            yawRateDps = 0.0f,
            isAvailable = true,
        )

        val result = compensator.compensate(
            vectors = vectors,
            imu = imu,
            dtSec = 0.1f,
            imageWidth = 640,
            imageHeight = 360,
        )

        assertEquals(1.0f, result.confidenceMultiplier, 0.001f)
        assertFalse(result.isRotationDominant)
        assertEquals(0.0f, result.estimatedRotationalDx, 0.001f)
        assertEquals(5.0f, result.compensatedVectors[0].dx, 0.01f)
        assertEquals(2.0f, result.compensatedVectors[0].dy, 0.01f)
    }

    @Test
    fun `pure yaw rotation cancels rotational displacement from observed flow`() {
        // Yaw rate = +0.2 rad/s (~11.5 deg/s)
        // With f = 550, dt = 0.1s:
        // Expected rotational dx = 550 * 0.2 * 0.1 = 11.0 px
        val yawRad = 0.2f
        val imu = ImuReading(
            gyroZ = yawRad,
            isAvailable = true,
        )

        // An observed feature that only moved due to camera rotation (dx = 11.0 px)
        val vectors = listOf(
            FlowVector(320f, 180f, 331f, 180f, 0.9f, true)
        )

        val result = compensator.compensate(
            vectors = vectors,
            imu = imu,
            dtSec = 0.1f,
            imageWidth = 640,
            imageHeight = 360,
        )

        assertEquals(11.0f, result.estimatedRotationalDx, 0.1f)
        // Compensated dx should be ~0.0 px (pure rotation cancelled)
        assertEquals(0.0f, result.compensatedVectors[0].dx, 0.2f)
        assertEquals(1.0f, result.confidenceMultiplier, 0.01f)
    }

    @Test
    fun `severe rotation above 45 deg per sec damps confidence factor heavily`() {
        // 50 deg/s in radians ≈ 0.872 rad/s
        val yawRad = Math.toRadians(50.0).toFloat()
        val imu = ImuReading(
            gyroZ = yawRad,
            isAvailable = true,
        )

        val vectors = listOf(
            FlowVector(320f, 180f, 360f, 180f, 0.9f, true)
        )

        val result = compensator.compensate(
            vectors = vectors,
            imu = imu,
            dtSec = 0.1f,
            imageWidth = 640,
            imageHeight = 360,
        )

        assertTrue("Rotation should be dominant", result.isRotationDominant)
        assertTrue("Confidence multiplier should be <= 0.1", result.confidenceMultiplier <= 0.1f)
        assertTrue("Vector confidence must be damped", result.compensatedVectors[0].confidence < 0.1f)
    }

    @Test
    fun `null IMU returns raw vectors safely with full multiplier`() {
        val vectors = listOf(FlowVector(10f, 10f, 12f, 12f, 0.8f, true))
        val result = compensator.compensate(vectors, null, 0.1f, 640, 360)

        assertEquals(1.0f, result.confidenceMultiplier, 0.001f)
        assertFalse(result.isRotationDominant)
        assertEquals(2.0f, result.compensatedVectors[0].dx, 0.001f)
    }

    @Test
    fun `pitch tilt down produces negative rotational dy and cancels observed upward displacement`() {
        // Pitch rate = +0.2 rad/s (nose down)
        // With f = 550, dt = 0.1s:
        // Expected bulkRotDy = -550 * 0.2 * 0.1 = -11.0 px
        val pitchRad = 0.2f
        val imu = ImuReading(
            gyroX = pitchRad,
            gyroZ = 0.0f,
            isAvailable = true,
        )

        // Scene shifted up due to camera tilting down (dy = -11.0 px)
        val vectors = listOf(
            FlowVector(320f, 180f, 320f, 169f, 0.9f, true)
        )

        val result = compensator.compensate(
            vectors = vectors,
            imu = imu,
            dtSec = 0.1f,
            imageWidth = 640,
            imageHeight = 360,
        )

        assertEquals(-11.0f, result.estimatedRotationalDy, 0.1f)
        assertEquals(0.0f, result.compensatedVectors[0].dy, 0.2f)
    }
}
