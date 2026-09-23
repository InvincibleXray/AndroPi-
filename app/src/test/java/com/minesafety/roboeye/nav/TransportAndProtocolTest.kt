package com.minesafety.roboeye.nav

import com.minesafety.roboeye.core.model.MotionCommand
import com.minesafety.roboeye.core.model.SafetyState
import com.minesafety.roboeye.esp32.transport.MockTransport
import com.minesafety.roboeye.localization.LocalPose
import com.minesafety.roboeye.localization.TrackingQuality
import com.minesafety.roboeye.mapping.LocalSpatialMap
import com.minesafety.roboeye.mapping.MapCellState
import com.minesafety.roboeye.nav.model.LocalGoal
import com.minesafety.roboeye.nav.model.NavigationWorldModel
import com.minesafety.roboeye.nav.state.NavigationState
import com.minesafety.roboeye.nav.transport.RoverCommandSender
import com.minesafety.roboeye.vision.GeometryTrustLevel
import kotlinx.coroutines.runBlocking
import kotlinx.serialization.json.Json
import kotlinx.serialization.json.jsonObject
import kotlinx.serialization.json.jsonPrimitive
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertTrue
import org.junit.Before
import org.junit.Test

/**
 * Deterministic unit tests covering:
 * T. Command timeout & keepalive
 * U. Invalid command parameter rejection
 * V. Monotonic command sequence numbers
 * W. Heartbeat transmission
 * Y. Mock transport simulation (end-to-end navigator execution)
 * Z. ESP32 protocol compatibility & JSON frame validation
 */
class TransportAndProtocolTest {

    private lateinit var mockTransport: MockTransport
    private lateinit var sender: RoverCommandSender

    @Before
    fun setUp() = runBlocking {
        mockTransport = MockTransport()
        mockTransport.open()
        sender = RoverCommandSender(mockTransport, minCommandIntervalMs = 50L, heartbeatIntervalMs = 1000L)
    }

    // --- U. INVALID COMMAND REJECTION TESTS ---
    @Test
    fun testCommandSender_RejectsNanAndInfiniteVelocities() = runBlocking {
        val nanCmd = MotionCommand(linearVelocityMps = Float.NaN)
        val success = sender.sendCommand(nanCmd)

        assertFalse("Command sender must reject NaN velocities", success)
        // Should transmit emergency/failsafe stop
        assertTrue("Sender must transmit safe stop upon rejecting invalid command", mockTransport.sentRawLines.isNotEmpty())
        val lastLine = mockTransport.sentRawLines.last()
        assertTrue(lastLine.contains("\"STOP\""))
    }

    @Test
    fun testCommandSender_RejectsOutOfRangeSpeeds() = runBlocking {
        val excessiveCmd = MotionCommand(linearVelocityMps = 5.0f) // > 1.0 m/s limit
        val success = sender.sendCommand(excessiveCmd)

        assertFalse("Command sender must reject out-of-range speed", success)
    }

    // --- V. MONOTONIC SEQUENCE NUMBER TESTS ---
    @Test
    fun testCommandSender_MonotonicallyIncrementsSequenceNumbers() = runBlocking {
        sender.sendCommand(MotionCommand(linearVelocityMps = 0.20f), force = true)
        sender.sendCommand(MotionCommand(linearVelocityMps = 0.25f), force = true)

        assertTrue(mockTransport.sentRawLines.size >= 2)

        val json1 = Json.parseToJsonElement(mockTransport.sentRawLines[0]).jsonObject
        val json2 = Json.parseToJsonElement(mockTransport.sentRawLines[1]).jsonObject

        val seq1 = json1["seq"]?.jsonPrimitive?.content?.toLongOrNull() ?: 0L
        val seq2 = json2["seq"]?.jsonPrimitive?.content?.toLongOrNull() ?: 0L

        assertTrue("Sequence numbers must be monotonically increasing ($seq2 > $seq1)", seq2 > seq1)
    }

    // --- T & W. HEARTBEAT / KEEPALIVE TESTS ---
    @Test
    fun testHeartbeat_TransmitsWhenIdleExceedsInterval() = runBlocking {
        val t0 = 1000L
        sender.sendCommand(MotionCommand.STOP, nowMs = t0, force = true)
        mockTransport.sentRawLines.clear()

        // 500ms later: within 1000ms heartbeat interval -> no ping sent
        val sentEarly = sender.maybeSendHeartbeat(nowMs = t0 + 500L)
        assertFalse(sentEarly)
        assertTrue(mockTransport.sentRawLines.isEmpty())

        // 1100ms later: exceeds 1000ms heartbeat interval -> ping must be sent to satisfy ESP32 watchdog
        val sentLate = sender.maybeSendHeartbeat(nowMs = t0 + 1100L)
        assertTrue("Heartbeat must be sent after interval elapsed", sentLate)
        assertEquals(1, mockTransport.sentRawLines.size)
        assertTrue(mockTransport.sentRawLines[0].contains("\"ping\""))
    }

    // --- Z. ESP32 PROTOCOL COMPATIBILITY TESTS ---
    @Test
    fun testProtocolCompatibility_JsonMatchesFirmwareSchema() = runBlocking {
        sender.sendCommand(MotionCommand.EMERGENCY_STOP, nowMs = 1726612800000L, force = true)

        val lastLine = mockTransport.sentRawLines.last()
        val doc = Json.parseToJsonElement(lastLine).jsonObject

        // Firmware expects command/type, value, and optionally seq, timestamp_ms
        assertEquals("EMERGENCY_STOP", doc["command"]?.jsonPrimitive?.content)
        assertEquals("EMERGENCY_STOP", doc["type"]?.jsonPrimitive?.content)
        val valueFloat: Float = doc["value"]?.jsonPrimitive?.content?.toFloatOrNull() ?: 0f
        assertEquals(1.0f, valueFloat, 0.01f)
        assertNotNull(doc["seq"])
        assertNotNull(doc["timestamp_ms"])
    }

    // --- Y. MOCK TRANSPORT END-TO-END SIMULATION TEST ---
    @Test
    fun testMockSimulation_AutonomousNavigatorExecutesRoute() = runBlocking {
        val map = LocalSpatialMap()
        // Clear 2m path ahead
        for (gx in 10..22) {
            for (gy in 10..14) {
                map.cells[gx][gy].state = MapCellState.FREE
                map.cells[gx][gy].confidence = 0.95f
            }
        }

        val navigator = AutonomousNavigator(commandSender = sender)
        val initialPose = LocalPose(xM = 0.0f, yM = 0.0f, yawDeg = 0.0f, trackingState = TrackingQuality.TRACKING)
        val worldModel = NavigationWorldModel(
            pose = initialPose,
            spatialMap = map,
            geometryTrust = GeometryTrustLevel.TRUSTED,
            timestampMs = System.currentTimeMillis(),
        )

        // Set local goal at 1.0m ahead
        val goal = LocalGoal(targetX = 1.0f, targetY = 0.0f)
        val goalResult = navigator.setGoal(goal, worldModel)
        assertTrue("Goal should be accepted", goalResult.isValid)
        assertEquals(NavigationState.READY, navigator.stateMachine.currentState)

        // Run navigation tick
        val arbitrated = navigator.tick(
            worldModel = worldModel,
            safetyState = SafetyState.NOMINAL,
            isTransportConnected = true,
        )

        // Verify navigator transitions to active NAVIGATING and generates forward motion
        assertEquals(NavigationState.NAVIGATING, navigator.stateMachine.currentState)
        assertTrue("Linear speed should be positive", arbitrated.command.linearVelocityMps > 0f)
        assertTrue("Raw lines sent across mock link should contain motor command", mockTransport.sentRawLines.isNotEmpty())
    }
}
