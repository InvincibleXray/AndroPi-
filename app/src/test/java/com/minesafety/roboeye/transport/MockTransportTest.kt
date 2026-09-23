package com.minesafety.roboeye.transport

import com.minesafety.roboeye.core.model.Esp32SensorState
import com.minesafety.roboeye.core.model.MotionCommand
import com.minesafety.roboeye.esp32.transport.MockTransport
import com.minesafety.roboeye.robot.transport.TransportStatus
import com.minesafety.roboeye.robot.transport.TransportType
import kotlinx.coroutines.flow.first
import kotlinx.coroutines.test.runTest
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Test

class MockTransportTest {

    @Test
    fun transportLifecycle_connectsAndDisconnectsCleanly() = runTest {
        val transport = MockTransport()
        assertEquals(TransportType.MOCK, transport.type)
        assertEquals(TransportStatus.Disconnected, transport.status.value)
        assertFalse(transport.status.value.isConnected)

        val connected = transport.open()
        assertTrue(connected)
        assertTrue(transport.status.value.isConnected)
        assertTrue(transport.status.value is TransportStatus.Connected)

        transport.close()
        assertEquals(TransportStatus.Disconnected, transport.status.value)
        assertFalse(transport.status.value.isConnected)
    }

    @Test
    fun sendCommand_recordsSentCommands() = runTest {
        val transport = MockTransport()
        transport.open()

        val cmd1 = MotionCommand(linearVelocityMps = 0.5f, angularVelocityRadS = 0.2f)
        val cmd2 = MotionCommand.EMERGENCY_STOP

        assertTrue(transport.sendCommand(cmd1))
        assertTrue(transport.sendCommand(cmd2))

        assertEquals(2, transport.sentCommands.size)
        assertEquals(cmd1, transport.sentCommands[0])
        assertEquals(cmd2, transport.sentCommands[1])
    }

    @Test
    fun telemetryEmission_emitsToFlowSubscribers() = runTest {
        val transport = MockTransport()
        val expected = Esp32SensorState(
            ultrasonicFrontM = 1.25f,
            ultrasonicLeftM = 0.8f,
            batteryVoltageV = 12.4f,
            hardwareEStop = false,
        )

        transport.emitTelemetry(expected)
        val received = transport.telemetry.first()

        assertEquals(1.25f, received.ultrasonicFrontM)
        assertEquals(0.8f, received.ultrasonicLeftM)
        assertEquals(12.4f, received.batteryVoltageV)
        assertFalse(received.hardwareEStop)
    }
}
