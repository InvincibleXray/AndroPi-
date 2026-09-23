package com.minesafety.roboeye.esp32.transport

import com.minesafety.roboeye.bridge.BridgeCodec
import com.minesafety.roboeye.bridge.BridgeParse
import com.minesafety.roboeye.bridge.LinkStatus
import com.minesafety.roboeye.bridge.UsbSerialLink
import com.minesafety.roboeye.core.model.Esp32SensorState
import com.minesafety.roboeye.core.model.MotionCommand
import com.minesafety.roboeye.robot.transport.RobotTransport
import com.minesafety.roboeye.robot.transport.TransportStatus
import com.minesafety.roboeye.robot.transport.TransportType
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.SupervisorJob
import kotlinx.coroutines.flow.Flow
import kotlinx.coroutines.flow.MutableSharedFlow
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asSharedFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.launch

/**
 * Production USB-OTG transport implementing [RobotTransport].
 *
 * Wraps the verified [UsbSerialLink] driver and [BridgeCodec] decoder without
 * exposing raw USB dependencies to the rest of the application.
 */
class UsbTransport(
    private val link: UsbSerialLink,
    private val scope: CoroutineScope = CoroutineScope(SupervisorJob() + Dispatchers.Default),
) : RobotTransport {

    override val name: String = "ESP32 USB-OTG"
    override val type: TransportType = TransportType.USB

    private val _status = MutableStateFlow<TransportStatus>(TransportStatus.Disconnected)
    override val status: StateFlow<TransportStatus> = _status.asStateFlow()

    private val _telemetry = MutableSharedFlow<Esp32SensorState>(extraBufferCapacity = 16)
    override val telemetry: Flow<Esp32SensorState> = _telemetry.asSharedFlow()

    override val rawLines: Flow<String> = link.lines

    init {
        // Map LinkStatus to universal TransportStatus
        scope.launch {
            link.status.collect { s ->
                _status.value = when (s) {
                    LinkStatus.Disconnected, LinkStatus.NoDevice, LinkStatus.NoPermission ->
                        TransportStatus.Disconnected
                    LinkStatus.Connecting -> TransportStatus.Connecting
                    is LinkStatus.Connected -> TransportStatus.Connected(s.deviceName)
                    is LinkStatus.Error -> TransportStatus.Error(s.message)
                }
            }
        }

        // Parse incoming NDJSON lines into domain Esp32SensorState
        scope.launch {
            link.lines.collect { line ->
                val now = System.currentTimeMillis()
                when (val result = BridgeCodec.parse(line, now)) {
                    is BridgeParse.Ok -> {
                        val t = result.telemetry
                        val state = Esp32SensorState(
                            ultrasonicFrontM = t.ultrasonicFront,
                            ultrasonicLeftM = t.ultrasonicLeft,
                            ultrasonicRightM = t.ultrasonicRight,
                            irLeft = t.irLeft,
                            irCenter = t.irCenter,
                            irRight = t.irRight,
                            chassisImuPitchDeg = t.imuPitchDeg,
                            chassisImuRollDeg = t.imuRollDeg,
                            hardwareEStop = t.estop ?: false,
                            uptimeMs = t.uptimeMs,
                            timestampMs = now,
                        )
                        _telemetry.tryEmit(state)
                    }
                    is BridgeParse.Notice, is BridgeParse.Rejected -> {
                        // Diagnostic logs preserved in rawLines
                    }
                }
            }
        }
    }

    override suspend fun open(): Boolean = link.open()

    override suspend fun sendCommand(command: MotionCommand): Boolean {
        if (command.emergencyStop) {
            val encoded = BridgeCodec.encodeCommand("ESTOP", 1.0f)
            return link.write(encoded)
        }
        // Send speed command
        val speedOk = link.write(BridgeCodec.encodeCommand("SPEED", command.linearVelocityMps))
        // Send steer command if non-zero
        val steerOk = if (command.angularVelocityRadS != 0.0f) {
            link.write(BridgeCodec.encodeCommand("STEER", command.angularVelocityRadS))
        } else true

        return speedOk && steerOk
    }

    override suspend fun sendRaw(line: String): Boolean = link.write(line)

    override fun close() = link.close()
}
