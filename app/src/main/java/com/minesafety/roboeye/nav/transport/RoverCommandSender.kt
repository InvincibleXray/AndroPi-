package com.minesafety.roboeye.nav.transport

import com.minesafety.roboeye.bridge.BridgeCodec
import com.minesafety.roboeye.core.model.MotionCommand
import com.minesafety.roboeye.robot.transport.RobotTransport
import kotlinx.coroutines.sync.Mutex
import kotlinx.coroutines.sync.withLock

/**
 * Validating transport sender managing command sequencing, timestamping,
 * rate limiting, and keepalive heartbeats for the ESP32 rover chassis.
 */
class RoverCommandSender(
    val transport: RobotTransport,
    private val minCommandIntervalMs: Long = 100L, // Max 10 Hz command rate
    private val heartbeatIntervalMs: Long = 1_000L, // 1 Hz keepalive to satisfy 5s watchdog
) {
    private val mutex = Mutex()
    private var sequenceNumber: Long = 1L
    private var lastCommandSentMs: Long = 0L
    private var lastSentCommand: MotionCommand? = null

    val currentSeq: Long get() = sequenceNumber

    /**
     * Sends a validated [MotionCommand] across the transport with sequence number and timestamp.
     */
    suspend fun sendCommand(
        command: MotionCommand,
        nowMs: Long = System.currentTimeMillis(),
        force: Boolean = false,
    ): Boolean = mutex.withLock {
        // Immediate priority for EMERGENCY_STOP (never rate limited)
        if (!command.emergencyStop && !force) {
            val elapsed = nowMs - lastCommandSentMs
            if (elapsed < minCommandIntervalMs) {
                // Rate limited: skip redundant interim frames if command is identical
                if (lastSentCommand?.linearVelocityMps == command.linearVelocityMps &&
                    lastSentCommand?.angularVelocityRadS == command.angularVelocityRadS
                ) {
                    return true
                }
            }
        }

        // Validate command parameters
        if (!isValidCommand(command)) {
            // Rejection failsafe: stop motors
            val stopJson = BridgeCodec.encodeCommand("STOP", 0.0f, sequenceNumber++, nowMs)
            transport.sendRaw(stopJson)
            return false
        }

        val success: Boolean

        if (command.emergencyStop) {
            val json = BridgeCodec.encodeCommand("EMERGENCY_STOP", 1.0f, sequenceNumber++, nowMs)
            success = transport.sendRaw(json)
        } else if (command.linearVelocityMps == 0.0f && command.angularVelocityRadS == 0.0f) {
            val json = BridgeCodec.encodeCommand("STOP", 0.0f, sequenceNumber++, nowMs)
            success = transport.sendRaw(json)
        } else {
            // Active motion command: set speed then direction
            val speedJson = BridgeCodec.encodeCommand("SET_SPEED", Math.abs(command.linearVelocityMps), sequenceNumber++, nowMs)
            transport.sendRaw(speedJson)

            val dirCmd = when {
                command.linearVelocityMps > 0.05f && Math.abs(command.angularVelocityRadS) <= 0.20f -> "MOVE_FORWARD"
                command.linearVelocityMps < -0.05f && Math.abs(command.angularVelocityRadS) <= 0.20f -> "MOVE_BACKWARD"
                command.angularVelocityRadS > 0.20f -> "TURN_LEFT"
                command.angularVelocityRadS < -0.20f -> "TURN_RIGHT"
                command.linearVelocityMps > 0f -> "MOVE_FORWARD"
                else -> "STOP"
            }
            val dirJson = BridgeCodec.encodeCommand(dirCmd, command.linearVelocityMps, sequenceNumber++, nowMs)
            success = transport.sendRaw(dirJson)
        }

        if (success) {
            lastCommandSentMs = nowMs
            lastSentCommand = command
        }
        return success
    }

    /**
     * Checks if a keepalive heartbeat is needed to prevent the ESP32 hardware watchdog (5s) from tripping.
     */
    suspend fun maybeSendHeartbeat(nowMs: Long = System.currentTimeMillis()): Boolean = mutex.withLock {
        if (nowMs - lastCommandSentMs >= heartbeatIntervalMs) {
            val pingJson = BridgeCodec.encodeCommand("ping", 0.0f, sequenceNumber++, nowMs)
            val ok = transport.sendRaw(pingJson)
            if (ok) lastCommandSentMs = nowMs
            return ok
        }
        return false
    }

    private fun isValidCommand(cmd: MotionCommand): Boolean {
        if (cmd.linearVelocityMps.isNaN() || cmd.linearVelocityMps.isInfinite()) return false
        if (cmd.angularVelocityRadS.isNaN() || cmd.angularVelocityRadS.isInfinite()) return false
        if (cmd.linearVelocityMps !in -1.0f..1.0f) return false
        if (cmd.angularVelocityRadS !in -2.0f..2.0f) return false
        return true
    }
}
