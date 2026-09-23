package com.minesafety.roboeye.control

import com.minesafety.roboeye.control.safety.SafetyController
import com.minesafety.roboeye.core.config.RoverBrainConfig
import com.minesafety.roboeye.core.ImuReading
import com.minesafety.roboeye.core.model.Esp32SensorState
import com.minesafety.roboeye.core.model.MotionCommand
import com.minesafety.roboeye.core.model.PhoneSensorState
import com.minesafety.roboeye.core.model.SafetyLevel
import com.minesafety.roboeye.core.model.SensorStatus
import com.minesafety.roboeye.esp32.transport.MockTransport
import kotlinx.coroutines.test.runTest
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Before
import org.junit.Test

class SafetyControllerTest {

    private lateinit var transport: MockTransport
    private lateinit var safetyController: SafetyController

    @Before
    fun setUp() {
        transport = MockTransport()
        safetyController = SafetyController(
            transport = transport,
            config = RoverBrainConfig(
                maxTiltDeg = 25.0f,
                eStopDistanceM = 0.5f,
                slowDownDistanceM = 1.5f,
                minVisibilityScore = 0.10f,
            )
        )
    }

    @Test
    fun disconnectedTransport_forcesEmergencyStop() {
        val verdict = safetyController.evaluate(
            esp32 = Esp32SensorState(ultrasonicFrontM = 2.0f),
            phone = null,
            visibilityScore = 0.8f,
            isTransportConnected = false,
        )

        assertEquals(SafetyLevel.EMERGENCY_STOP, verdict.level)
        assertTrue(verdict.isEmergencyStop)
        assertEquals(0.0f, verdict.speedLimitMps, 0.001f)
        assertTrue(verdict.reasons.any { it.contains("disconnected", ignoreCase = true) })
    }

    @Test
    fun criticalObstacleProximity_triggersEmergencyStop() {
        val verdict = safetyController.evaluate(
            esp32 = Esp32SensorState(ultrasonicFrontM = 0.35f), // < 0.5m
            phone = null,
            visibilityScore = 0.8f,
            isTransportConnected = true,
        )

        assertEquals(SafetyLevel.EMERGENCY_STOP, verdict.level)
        assertTrue(verdict.isEmergencyStop)
        assertTrue(verdict.reasons.any { it.contains("critical proximity", ignoreCase = true) })
    }

    @Test
    fun cautionObstacleProximity_reducesSpeedLimit() {
        val verdict = safetyController.evaluate(
            esp32 = Esp32SensorState(ultrasonicFrontM = 1.1f), // 0.5m < dist < 1.5m
            phone = null,
            visibilityScore = 0.8f,
            isTransportConnected = true,
        )

        assertEquals(SafetyLevel.CAUTION, verdict.level)
        assertFalse(verdict.isEmergencyStop)
        assertTrue(verdict.speedLimitMps <= 0.35f)
    }

    @Test
    fun excessiveVehicleTilt_triggersRolloverEmergencyStop() {
        val verdict = safetyController.evaluate(
            esp32 = null,
            phone = PhoneSensorState(
                imuStatus = SensorStatus.ACTIVE,
                imu = ImuReading(axG = 0f, ayG = 0.5f, azG = 0.8f, pitchDeg = 28.0f, rollDeg = 5.0f) // > 25°
            ),
            visibilityScore = 0.9f,
            isTransportConnected = true,
        )

        assertEquals(SafetyLevel.EMERGENCY_STOP, verdict.level)
        assertTrue(verdict.reasons.any { it.contains("rollover", ignoreCase = true) })
    }

    @Test
    fun zeroAtmosphericVisibility_triggersEmergencyStop() {
        val verdict = safetyController.evaluate(
            esp32 = Esp32SensorState(ultrasonicFrontM = 3.0f),
            phone = null,
            visibilityScore = 0.05f, // < 0.10
            isTransportConnected = true,
        )

        assertEquals(SafetyLevel.EMERGENCY_STOP, verdict.level)
        assertTrue(verdict.reasons.any { it.contains("visibility", ignoreCase = true) })
    }

    @Test
    fun nominalClearConditions_permitFullSpeedMotion() = runTest {
        transport.open()
        val proposed = MotionCommand(linearVelocityMps = 0.8f, angularVelocityRadS = 0.0f)

        val success = safetyController.dispatchCommand(
            proposed = proposed,
            esp32 = Esp32SensorState(ultrasonicFrontM = 3.5f),
            phone = PhoneSensorState(
                imuStatus = SensorStatus.ACTIVE,
                imu = ImuReading(axG = 0f, ayG = 0f, azG = 1f, pitchDeg = 2.0f, rollDeg = 1.0f)
            ),
            visibilityScore = 0.85f,
        )

        assertTrue(success)
        assertEquals(1, transport.sentCommands.size)
        val sent = transport.sentCommands.first()
        assertEquals(0.8f, sent.linearVelocityMps, 0.001f)
        assertFalse(sent.emergencyStop)
    }

    @Test
    fun cautionCondition_clampsForwardVelocity() = runTest {
        transport.open()
        val highSpeedProposed = MotionCommand(linearVelocityMps = 1.0f, angularVelocityRadS = 0.0f)

        val success = safetyController.dispatchCommand(
            proposed = highSpeedProposed,
            esp32 = Esp32SensorState(ultrasonicFrontM = 1.0f), // Caution zone
            visibilityScore = 0.7f,
        )

        assertTrue(success)
        val sent = transport.sentCommands.first()
        assertTrue(sent.linearVelocityMps <= 0.35f)
        assertFalse(sent.emergencyStop)
    }

    @Test
    fun activeEStop_overridesProposedMotionToEStop() = runTest {
        transport.open()
        val highSpeedProposed = MotionCommand(linearVelocityMps = 1.0f, angularVelocityRadS = 0.0f)

        val success = safetyController.dispatchCommand(
            proposed = highSpeedProposed,
            esp32 = Esp32SensorState(ultrasonicFrontM = 0.2f), // < 0.5m critical
            visibilityScore = 0.7f,
        )

        assertTrue(success)
        val sent = transport.sentCommands.first()
        assertTrue(sent.emergencyStop)
    }
}
