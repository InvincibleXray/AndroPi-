package com.minesafety.roboeye.esp32.transport

import com.minesafety.roboeye.core.model.Esp32SensorState
import com.minesafety.roboeye.core.model.MotionCommand
import com.minesafety.roboeye.robot.transport.RobotTransport
import com.minesafety.roboeye.robot.transport.TransportStatus
import com.minesafety.roboeye.robot.transport.TransportType
import kotlinx.coroutines.flow.Flow
import kotlinx.coroutines.flow.MutableSharedFlow
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asSharedFlow
import kotlinx.coroutines.flow.asStateFlow

/**
 * Deterministic in-memory simulated transport for offline JVM tests and desktop emulation.
 */
class MockTransport : RobotTransport {

    override val name: String = "Mock Simulated Link"
    override val type: TransportType = TransportType.MOCK

    private val _status = MutableStateFlow<TransportStatus>(TransportStatus.Disconnected)
    override val status: StateFlow<TransportStatus> = _status.asStateFlow()

    private val _rawLines = MutableSharedFlow<String>(extraBufferCapacity = 64)
    override val rawLines: Flow<String> = _rawLines.asSharedFlow()

    private val _telemetry = MutableSharedFlow<Esp32SensorState>(replay = 1, extraBufferCapacity = 64)
    override val telemetry: Flow<Esp32SensorState> = _telemetry.asSharedFlow()

    val sentCommands = mutableListOf<MotionCommand>()
    val sentRawLines = mutableListOf<String>()

    override suspend fun open(): Boolean {
        _status.value = TransportStatus.Connected("Virtual Chassis Simulator")
        return true
    }

    override suspend fun sendCommand(command: MotionCommand): Boolean {
        sentCommands.add(command)
        return true
    }

    override suspend fun sendRaw(line: String): Boolean {
        sentRawLines.add(line)
        return true
    }

    /** Emits a synthetic telemetry update for test fixtures. */
    fun emitTelemetry(state: Esp32SensorState) {
        _telemetry.tryEmit(state)
    }

    override fun close() {
        _status.value = TransportStatus.Disconnected
    }
}
