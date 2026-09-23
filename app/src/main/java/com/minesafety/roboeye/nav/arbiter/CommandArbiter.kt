package com.minesafety.roboeye.nav.arbiter

import com.minesafety.roboeye.core.model.MotionCommand
import com.minesafety.roboeye.core.model.SafetyLevel
import com.minesafety.roboeye.core.model.SafetyState
import com.minesafety.roboeye.localization.TrackingQuality

/**
 * Strict priority tiers for vehicle control commands.
 */
enum class CommandPriority {
    PRIORITY_1_HARDWARE_ESTOP,
    PRIORITY_2_SAFETY_FAULT,
    PRIORITY_3_COMMUNICATION_FAILURE,
    PRIORITY_4_LOCALIZATION_FAILURE,
    PRIORITY_5_MANUAL_EMERGENCY_OVERRIDE,
    PRIORITY_6_AUTONOMOUS_SAFETY_STOP,
    PRIORITY_7_AUTONOMOUS_MOTION,
    PRIORITY_8_IDLE,
}

/**
 * The single authoritative result of command arbitration.
 */
data class ArbitratedCommand(
    val command: MotionCommand,
    val activePriority: CommandPriority,
    val reason: String,
    val timestampMs: Long = System.currentTimeMillis(),
)

/**
 * Single authoritative multiplexer ensuring no lower-priority motion command
 * can ever override a higher-priority safety or fault state.
 */
class CommandArbiter {

    fun arbitrate(
        hardwareEStop: Boolean,
        safetyState: SafetyState,
        isTransportConnected: Boolean,
        localizationQuality: TrackingQuality,
        manualEmergencyStop: Boolean,
        manualCommand: MotionCommand? = null,
        autonomousCommand: MotionCommand? = null,
        nowMs: Long = System.currentTimeMillis(),
    ): ArbitratedCommand {
        // 1. HARDWARE E-STOP
        if (hardwareEStop) {
            return ArbitratedCommand(
                command = MotionCommand.EMERGENCY_STOP.copy(
                    source = "HARDWARE_ESTOP",
                    timestampNs = nowMs * 1_000_000L,
                ),
                activePriority = CommandPriority.PRIORITY_1_HARDWARE_ESTOP,
                reason = "Hardware E-Stop button active on chassis",
                timestampMs = nowMs,
            )
        }

        // 2. SAFETY FAULT (tilt, critical proximity, zero visibility)
        if (safetyState.level == SafetyLevel.EMERGENCY_STOP) {
            val reason = safetyState.reasons.firstOrNull() ?: "Critical safety limit breached"
            return ArbitratedCommand(
                command = MotionCommand.EMERGENCY_STOP.copy(
                    source = "SAFETY_FAULT",
                    timestampNs = nowMs * 1_000_000L,
                ),
                activePriority = CommandPriority.PRIORITY_2_SAFETY_FAULT,
                reason = reason,
                timestampMs = nowMs,
            )
        }

        // 3. COMMUNICATION FAILURE
        if (!isTransportConnected) {
            return ArbitratedCommand(
                command = MotionCommand.STOP.copy(
                    source = "COMM_FAILURE",
                    timestampNs = nowMs * 1_000_000L,
                ),
                activePriority = CommandPriority.PRIORITY_3_COMMUNICATION_FAILURE,
                reason = "ESP32 transport link disconnected",
                timestampMs = nowMs,
            )
        }

        // 4. LOCALIZATION FAILURE
        if (localizationQuality == TrackingQuality.LOST) {
            return ArbitratedCommand(
                command = MotionCommand.STOP.copy(
                    source = "LOCALIZATION_LOST",
                    timestampNs = nowMs * 1_000_000L,
                ),
                activePriority = CommandPriority.PRIORITY_4_LOCALIZATION_FAILURE,
                reason = "Visual-inertial tracking quality is LOST",
                timestampMs = nowMs,
            )
        }

        // 5. MANUAL EMERGENCY OVERRIDE
        if (manualEmergencyStop) {
            return ArbitratedCommand(
                command = MotionCommand.EMERGENCY_STOP.copy(
                    source = "MANUAL_ESTOP",
                    timestampNs = nowMs * 1_000_000L,
                ),
                activePriority = CommandPriority.PRIORITY_5_MANUAL_EMERGENCY_OVERRIDE,
                reason = "Operator manual emergency stop triggered",
                timestampMs = nowMs,
            )
        }

        // Manual teleop command if provided by operator
        if (manualCommand != null && manualCommand.source.contains("MANUAL", ignoreCase = true)) {
            val clampedLinear = if (safetyState.level == SafetyLevel.CAUTION) {
                manualCommand.linearVelocityMps.coerceIn(-safetyState.speedLimitMps, safetyState.speedLimitMps)
            } else manualCommand.linearVelocityMps

            return ArbitratedCommand(
                command = manualCommand.copy(
                    linearVelocityMps = clampedLinear,
                    timestampNs = nowMs * 1_000_000L,
                ),
                activePriority = CommandPriority.PRIORITY_7_AUTONOMOUS_MOTION,
                reason = "Manual teleoperation command accepted",
                timestampMs = nowMs,
            )
        }

        // 6. AUTONOMOUS SAFETY STOP
        if (autonomousCommand != null &&
            autonomousCommand.linearVelocityMps == 0f &&
            autonomousCommand.angularVelocityRadS == 0f &&
            autonomousCommand.source != "IDLE"
        ) {
            return ArbitratedCommand(
                command = autonomousCommand.copy(timestampNs = nowMs * 1_000_000L),
                activePriority = CommandPriority.PRIORITY_6_AUTONOMOUS_SAFETY_STOP,
                reason = "Autonomous stop: ${autonomousCommand.source}",
                timestampMs = nowMs,
            )
        }

        // 7. AUTONOMOUS MOTION
        if (autonomousCommand != null) {
            val clampedLinear = if (safetyState.level == SafetyLevel.CAUTION) {
                autonomousCommand.linearVelocityMps.coerceIn(-safetyState.speedLimitMps, safetyState.speedLimitMps)
            } else autonomousCommand.linearVelocityMps

            return ArbitratedCommand(
                command = autonomousCommand.copy(
                    linearVelocityMps = clampedLinear,
                    timestampNs = nowMs * 1_000_000L,
                ),
                activePriority = CommandPriority.PRIORITY_7_AUTONOMOUS_MOTION,
                reason = "Autonomous navigation command accepted",
                timestampMs = nowMs,
            )
        }

        // 8. IDLE
        return ArbitratedCommand(
            command = MotionCommand.STOP.copy(source = "IDLE", timestampNs = nowMs * 1_000_000L),
            activePriority = CommandPriority.PRIORITY_8_IDLE,
            reason = "System idle; no motion active",
            timestampMs = nowMs,
        )
    }
}
