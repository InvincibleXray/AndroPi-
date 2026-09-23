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
 * Extension point for future direct Wi-Fi / UDP / WebSocket link between Rover Brain and ESP32.
 */
class WifiTransportStub(
    val host: String = "192.168.4.1",
    val port: Int = 8080,
) : RobotTransport {

    override val name: String = "ESP32 over Wi-Fi"
    override val type: TransportType = TransportType.WIFI

    private val _status = MutableStateFlow<TransportStatus>(TransportStatus.Disconnected)
    override val status: StateFlow<TransportStatus> = _status.asStateFlow()

    private val _rawLines = MutableSharedFlow<String>()
    override val rawLines: Flow<String> = _rawLines.asSharedFlow()

    private val _telemetry = MutableSharedFlow<Esp32SensorState>()
    override val telemetry: Flow<Esp32SensorState> = _telemetry.asSharedFlow()

    override suspend fun open(): Boolean {
        _status.value = TransportStatus.Error("Wi-Fi transport not configured (Phase 2 extension point)")
        return false
    }

    override suspend fun sendCommand(command: MotionCommand): Boolean = false

    override suspend fun sendRaw(line: String): Boolean = false

    override fun close() {
        _status.value = TransportStatus.Disconnected
    }
}
