package com.minesafety.roboeye.fusion

import com.minesafety.roboeye.core.ImuReading
import com.minesafety.roboeye.core.model.Esp32SensorState
import com.minesafety.roboeye.core.model.ObstacleObservation
import com.minesafety.roboeye.core.model.PhoneSensorState
import com.minesafety.roboeye.vision.VisionOutput
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertTrue
import org.junit.Test

class SensorFusionEngineTest {

    private val fusionEngine = DefaultSensorFusionEngine()

    @Test
    fun fuse_nominalForwardMotionWithObstacle_confirmsFusedState() {
        val now = System.currentTimeMillis()
        val phone = PhoneSensorState(
            imu = ImuReading(axG = 0.05f, yawRateDps = 2.0f),
            timestampMs = now,
        )
        val esp32 = Esp32SensorState(
            ultrasonicFrontM = 0.9f, // obstacle within caution range (<1.5m)
            ultrasonicLeftM = 3.0f,
            ultrasonicRightM = 3.0f,
            timestampMs = now,
        )
        val vision = VisionOutput(
            frameSequence = 10L,
            timestampNs = now * 1_000_000L,
            obstacles = listOf(
                ObstacleObservation(
                    id = "FLOW_1",
                    distanceM = 1.0f,
                    bearingDeg = 0.0f,
                    timeToCollisionSec = 1.8f,
                    source = "OPTICAL_FLOW",
                    confidence = 0.8f,
                )
            ),
        )

        val fused = fusionEngine.fuse(phone, esp32, vision)

        assertFalse("Center obstacle should block forward corridor", fused.isClearAhead)
        assertNotNull("Fused obstacle list must not be empty", fused.fusedObstacles)
        assertTrue("Obstacle list must contain confirmed center hazard", fused.fusedObstacles.isNotEmpty())
        assertEquals("FUSED_DUAL_RANGE_VISION", fused.fusedObstacles[0].source)
        assertEquals(0.9f, fused.fusedObstacles[0].distanceM, 0.001f)
    }

    @Test
    fun fuse_turningRover_suppressesVisualFalseAlarms() {
        val now = System.currentTimeMillis()
        // Strong turning motion (yawRate = 45°/s)
        val phone = PhoneSensorState(
            imu = ImuReading(yawRateDps = 45.0f),
            timestampMs = now,
        )
        val esp32 = Esp32SensorState(
            ultrasonicFrontM = 3.5f, // clear space ahead in reality
            ultrasonicLeftM = 3.5f,
            ultrasonicRightM = 3.5f,
            timestampMs = now,
        )
        // False visual collision expansion produced by turning camera
        val vision = VisionOutput(
            frameSequence = 12L,
            timestampNs = now * 1_000_000L,
            obstacles = listOf(
                ObstacleObservation(
                    id = "FALSE_TURN_FLOW",
                    distanceM = 0.8f,
                    bearingDeg = 0.0f,
                    timeToCollisionSec = 1.2f,
                    source = "OPTICAL_FLOW",
                    confidence = 0.9f,
                )
            ),
        )

        val fused = fusionEngine.fuse(phone, esp32, vision)

        assertTrue("Rotational motion flag must be set", fused.isRotationalMotion)
        // Visual false alarm is suppressed by gyro motion compensation, and physical sensor reports clear space ahead
        assertTrue("Forward corridor should remain clear ahead during turn without physical obstacle", fused.isClearAhead)
    }

    @Test
    fun fuse_staleSensorData_marksUnknownState() {
        val now = System.currentTimeMillis()
        val phone = PhoneSensorState(
            imu = ImuReading(yawRateDps = 0.0f),
            timestampMs = now - 2000L, // 2s old (stale)
        )
        val esp32 = Esp32SensorState(
            ultrasonicFrontM = 1.0f,
            timestampMs = now - 3000L, // 3s old (stale)
        )

        val fused = fusionEngine.fuse(phone, esp32, visionOutput = null)

        assertEquals(ReadingStatus.STALE, fused.sensorHealth["ULTRASONIC_FRONT"])
        assertEquals(ReadingStatus.STALE, fused.sensorHealth["PHONE_IMU"])
        // Unknown is not safe: stale sensor state must NOT report clear ahead
        assertFalse("Stale sensors must not declare clear ahead", fused.isClearAhead)
    }
}
