package com.minesafety.roboeye.control.safety

import com.minesafety.roboeye.core.Units
import com.minesafety.roboeye.core.config.RoverBrainConfig
import com.minesafety.roboeye.core.model.Esp32SensorState
import com.minesafety.roboeye.core.model.MotionCommand
import com.minesafety.roboeye.core.model.PhoneSensorState
import com.minesafety.roboeye.core.model.SafetyLevel
import com.minesafety.roboeye.core.model.SafetyState
import com.minesafety.roboeye.robot.transport.RobotTransport
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlin.math.abs

/**
 * Deterministic Safety Controller (The Ultimate Hardware Gatekeeper).
 *
 * Rules:
 * 1. Neither UI nor AI components may ever command the hardware transport directly.
 * 2. Every [MotionCommand] MUST pass through [dispatchCommand].
 * 3. If obstacle distance < eStopDistanceM (0.5m), chassis tilt > maxTiltDeg (25°),
 *    visibility < minVisibility (0.10), or link is disconnected:
 *    THE CONTROLLER OVERRIDES MOTOR COMMANDS TO EMERGENCY STOP.
 * 4. In CAUTION zone (obstacle < 1.5m), forward speed is deterministically clamped.
 */
class SafetyController(
    val transport: RobotTransport,
    val config: RoverBrainConfig = RoverBrainConfig(),
) {
    private val _safetyState = MutableStateFlow(SafetyState.NOMINAL)
    val safetyState: StateFlow<SafetyState> = _safetyState.asStateFlow()

    /**
     * Pure deterministic evaluation function, completely unit-testable on standard JVM.
     */
    fun evaluate(
        esp32: Esp32SensorState?,
        phone: PhoneSensorState?,
        visibilityScore: Float?,
        isTransportConnected: Boolean,
    ): SafetyState {
        val reasons = mutableListOf<String>()
        var level = SafetyLevel.NOMINAL
        var speedLimit = 1.0f

        fun escalate(newLevel: SafetyLevel, reason: String) {
            if (newLevel.ordinal > level.ordinal) level = newLevel
            reasons.add(reason)
        }

        // 1. Transport Link Integrity
        if (!isTransportConnected) {
            escalate(SafetyLevel.EMERGENCY_STOP, "Robot transport link is disconnected")
        }

        // 2. Hardware E-Stop button on chassis
        if (esp32?.hardwareEStop == true) {
            escalate(SafetyLevel.EMERGENCY_STOP, "Chassis hardware emergency stop switch triggered")
        }

        // 3. Proximity Obstacle Distances
        val minObstacle = esp32?.minObstacleM
        if (minObstacle != null) {
            when {
                minObstacle < config.eStopDistanceM -> {
                    escalate(
                        SafetyLevel.EMERGENCY_STOP,
                        "Obstacle critical proximity: ${"%.2f".format(minObstacle)} m < ${config.eStopDistanceM} m",
                    )
                }
                minObstacle < config.slowDownDistanceM -> {
                    escalate(
                        SafetyLevel.CAUTION,
                        "Obstacle caution proximity: ${"%.2f".format(minObstacle)} m < ${config.slowDownDistanceM} m",
                    )
                    speedLimit = speedLimit.coerceAtMost(0.35f)
                }
            }
        }

        // 4. Vehicle Tilt Angles (rollover protection)
        val pitch = esp32?.chassisImuPitchDeg ?: phone?.imu?.pitchDeg
        val roll = esp32?.chassisImuRollDeg ?: phone?.imu?.rollDeg
        if (pitch != null && roll != null) {
            val tilt = Units.tiltDeg(pitch, roll)
            if (tilt > config.maxTiltDeg) {
                escalate(
                    SafetyLevel.EMERGENCY_STOP,
                    "Excessive vehicle tilt: ${"%.1f".format(tilt)}° > ${config.maxTiltDeg}° (rollover hazard)",
                )
            }
        }

        // 5. Environmental Visibility
        if (visibilityScore != null && visibilityScore < config.minVisibilityScore) {
            escalate(
                SafetyLevel.EMERGENCY_STOP,
                "Zero atmospheric visibility score: ${"%.2f".format(visibilityScore)} < ${config.minVisibilityScore}",
            )
        }

        val verdict = SafetyState(
            level = level,
            isEmergencyStop = (level == SafetyLevel.EMERGENCY_STOP),
            speedLimitMps = if (level == SafetyLevel.EMERGENCY_STOP) 0.0f else speedLimit,
            reasons = reasons,
            minObstacleDistanceM = minObstacle,
            evaluatedAtMs = System.currentTimeMillis(),
        )

        _safetyState.value = verdict
        return verdict
    }

    /**
     * Vets and dispatches a proposed motion command through the safety policy gate.
     */
    suspend fun dispatchCommand(
        proposed: MotionCommand,
        esp32: Esp32SensorState? = null,
        phone: PhoneSensorState? = null,
        visibilityScore: Float? = null,
    ): Boolean {
        val verdict = evaluate(
            esp32 = esp32,
            phone = phone,
            visibilityScore = visibilityScore,
            isTransportConnected = transport.status.value.isConnected,
        )

        val safeCommand = when (verdict.level) {
            SafetyLevel.EMERGENCY_STOP -> {
                MotionCommand.EMERGENCY_STOP.copy(source = "SAFETY_GATE_ESTOP")
            }
            SafetyLevel.CAUTION -> {
                // Clamp forward velocity to safety speed limit
                val clampedLinear = proposed.linearVelocityMps.coerceIn(-verdict.speedLimitMps, verdict.speedLimitMps)
                proposed.copy(linearVelocityMps = clampedLinear)
            }
            SafetyLevel.NOMINAL -> {
                proposed
            }
        }

        return transport.sendCommand(safeCommand)
    }

    /** Forces an unconditional emergency stop on the hardware transport. */
    suspend fun emergencyStop(): Boolean {
        _safetyState.value = SafetyState(
            level = SafetyLevel.EMERGENCY_STOP,
            isEmergencyStop = true,
            speedLimitMps = 0.0f,
            reasons = listOf("Manual / Software emergency stop invoked"),
        )
        return transport.sendCommand(MotionCommand.EMERGENCY_STOP)
    }
}
