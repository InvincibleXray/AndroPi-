package com.minesafety.roboeye.robot.transport

import com.minesafety.roboeye.core.model.Esp32SensorState
import com.minesafety.roboeye.core.model.MotionCommand
import kotlinx.coroutines.flow.Flow
import kotlinx.coroutines.flow.StateFlow

/**
 * Universal transport abstraction for communicating between Rover Brain and the robot chassis / ESP32.
 *
 * Implementations:
 * - [com.minesafety.roboeye.esp32.transport.UsbTransport] (USB-OTG Serial CDC-ACM / UART)
 * - [com.minesafety.roboeye.esp32.transport.WifiTransportStub] (Wi-Fi / UDP / WebSockets)
 * - [com.minesafety.roboeye.esp32.transport.BluetoothTransportStub] (Bluetooth Serial / BLE)
 * - [com.minesafety.roboeye.esp32.transport.MockTransport] (Deterministic in-memory simulator for unit tests)
 */
interface RobotTransport {
    /** Human-readable transport name. */
    val name: String

    /** Underlying physical or virtual transport medium. */
    val type: TransportType

    /** Current link connection status. */
    val status: StateFlow<TransportStatus>

    /** Inbound raw lines from the hardware link. */
    val rawLines: Flow<String>

    /** Parsed domain telemetry stream emitted by the chassis. */
    val telemetry: Flow<Esp32SensorState>

    /** Opens the transport link. Returns true when connected. */
    suspend fun open(): Boolean

    /** Transmits a validated [MotionCommand] to the motor controller. */
    suspend fun sendCommand(command: MotionCommand): Boolean

    /** Transmits a raw string line over the wire (e.g. diagnostic / calibration command). */
    suspend fun sendRaw(line: String): Boolean

    /** Closes the transport link. */
    fun close()
}
