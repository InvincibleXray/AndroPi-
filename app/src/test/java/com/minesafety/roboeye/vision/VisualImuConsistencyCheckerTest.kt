package com.minesafety.roboeye.vision

import com.minesafety.roboeye.core.ImuReading
import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test

class VisualImuConsistencyCheckerTest {

    private val checker = VisualImuConsistencyChecker(
        stationaryFlowThresholdPx = 0.8f,
        stationaryGyroThresholdDps = 6.0f,
        movingFlowThresholdPx = 2.5f,
        movingGyroThresholdDps = 15.0f,
        minFeatureCount = 6,
    )

    private fun makeVectors(count: Int, dx: Float = 0f, dy: Float = 0f): List<FlowVector> {
        return (0 until count).map { i ->
            FlowVector(
                prevX = 100f + i * 5f,
                prevY = 100f + i * 5f,
                currX = 100f + i * 5f + dx,
                currY = 100f + i * 5f + dy,
                confidence = 0.9f,
                isValid = true,
            )
        }
    }

    @Test
    fun `both stationary produces CONSISTENT`() {
        val vectors = makeVectors(10, dx = 0.2f, dy = 0.1f)
        val imu = ImuReading(gyroZ = 0.01f, isAvailable = true) // ~0.5 deg/s

        val result = checker.check(vectors, meanDx = 0.2f, meanDy = 0.1f, imu = imu, isFreshImu = true)
        assertEquals(VisualImuConsistency.CONSISTENT, result.state)
        assertTrue(result.explanation.contains("stationary", ignoreCase = true))
    }

    @Test
    fun `consistent left turn produces CONSISTENT`() {
        // Camera flow moves right (+4.0 px) when turning left (+20 deg/s)
        val vectors = makeVectors(12, dx = 4.0f, dy = 0.0f)
        val gyroZRad = Math.toRadians(20.0).toFloat()
        val imu = ImuReading(gyroZ = gyroZRad, isAvailable = true)

        val result = checker.check(vectors, meanDx = 4.0f, meanDy = 0.0f, imu = imu, isFreshImu = true)
        assertEquals(VisualImuConsistency.CONSISTENT, result.state)
    }

    @Test
    fun `camera detects high flow but IMU reports stationary produces CONFLICTING`() {
        // High flow (+5.0 px) while gyro reads 0.0
        val vectors = makeVectors(12, dx = 5.0f, dy = 0.0f)
        val imu = ImuReading(gyroZ = 0.0f, isAvailable = true)

        val result = checker.check(vectors, meanDx = 5.0f, meanDy = 0.0f, imu = imu, isFreshImu = true)
        assertEquals(VisualImuConsistency.CONFLICTING, result.state)
        assertTrue(result.explanation.contains("Conflicting", ignoreCase = true))
    }

    @Test
    fun `gyro reports fast turn but camera flow is static produces CONFLICTING`() {
        val vectors = makeVectors(10, dx = 0.1f, dy = 0.1f)
        val gyroZRad = Math.toRadians(35.0).toFloat() // 35 deg/s
        val imu = ImuReading(gyroZ = gyroZRad, isAvailable = true)

        val result = checker.check(vectors, meanDx = 0.1f, meanDy = 0.1f, imu = imu, isFreshImu = true)
        assertEquals(VisualImuConsistency.CONFLICTING, result.state)
    }

    @Test
    fun `insufficient features produces INSUFFICIENT_DATA`() {
        val vectors = makeVectors(3, dx = 1.0f, dy = 1.0f)
        val imu = ImuReading(gyroZ = 0.0f, isAvailable = true)

        val result = checker.check(vectors, meanDx = 1.0f, meanDy = 1.0f, imu = imu, isFreshImu = true)
        assertEquals(VisualImuConsistency.INSUFFICIENT_DATA, result.state)
    }

    @Test
    fun `stale IMU produces INSUFFICIENT_DATA`() {
        val vectors = makeVectors(10, dx = 1.0f, dy = 1.0f)
        val imu = ImuReading(gyroZ = 0.0f, isAvailable = true)

        val result = checker.check(vectors, meanDx = 1.0f, meanDy = 1.0f, imu = imu, isFreshImu = false)
        assertEquals(VisualImuConsistency.INSUFFICIENT_DATA, result.state)
    }
}
