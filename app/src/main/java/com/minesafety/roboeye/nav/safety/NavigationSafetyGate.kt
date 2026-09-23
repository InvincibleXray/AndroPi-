package com.minesafety.roboeye.nav.safety

import com.minesafety.roboeye.core.model.MotionCommand
import com.minesafety.roboeye.core.model.SafetyLevel
import com.minesafety.roboeye.core.model.SafetyState
import com.minesafety.roboeye.localization.TrackingQuality
import com.minesafety.roboeye.nav.model.NavigationWorldModel
import com.minesafety.roboeye.vision.GeometryTrustLevel

enum class SafetyGateAction {
    ALLOW,
    SLOW,
    STOP,
    ESTOP,
}

data class SafetyGateVerdict(
    val safeCommand: MotionCommand,
    val action: SafetyGateAction,
    val reasons: List<String>,
    val timestampMs: Long = System.currentTimeMillis(),
)

/**
 * Deterministic safety gatekeeper independent from AI, planners, and UI.
 *
 * Ensures all safety invariants are strictly upheld before any command reaches the motor transport.
 */
class NavigationSafetyGate {

    fun vetCommand(
        proposed: MotionCommand,
        worldModel: NavigationWorldModel,
        safetyState: SafetyState,
        isTransportConnected: Boolean,
        nowMs: Long = System.currentTimeMillis(),
    ): SafetyGateVerdict {
        val reasons = mutableListOf<String>()

        // 1. Hardware / Platform Emergency Stop
        if (safetyState.isEmergencyStop || proposed.emergencyStop) {
            reasons.addAll(safetyState.reasons)
            if (proposed.emergencyStop && reasons.isEmpty()) reasons.add("Emergency stop commanded")
            return SafetyGateVerdict(
                safeCommand = MotionCommand.EMERGENCY_STOP.copy(
                    source = "SAFETY_GATE_ESTOP",
                    timestampNs = nowMs * 1_000_000L,
                ),
                action = SafetyGateAction.ESTOP,
                reasons = reasons,
                timestampMs = nowMs,
            )
        }

        // 2. Transport Link Loss
        if (!isTransportConnected) {
            return SafetyGateVerdict(
                safeCommand = MotionCommand.STOP.copy(
                    source = "LINK_DISCONNECTED",
                    timestampNs = nowMs * 1_000_000L,
                ),
                action = SafetyGateAction.STOP,
                reasons = listOf("Transport connection dropped"),
                timestampMs = nowMs,
            )
        }

        // 3. Localization Loss
        if (worldModel.pose.trackingState == TrackingQuality.LOST) {
            return SafetyGateVerdict(
                safeCommand = MotionCommand.STOP.copy(
                    source = "LOCALIZATION_LOST",
                    timestampNs = nowMs * 1_000_000L,
                ),
                action = SafetyGateAction.STOP,
                reasons = listOf("Visual-inertial tracking quality is LOST"),
                timestampMs = nowMs,
            )
        }

        // 4. Scene Geometry Untrusted
        if (worldModel.geometryTrust == GeometryTrustLevel.UNTRUSTED) {
            return SafetyGateVerdict(
                safeCommand = MotionCommand.STOP.copy(
                    source = "GEOMETRY_UNTRUSTED",
                    timestampNs = nowMs * 1_000_000L,
                ),
                action = SafetyGateAction.STOP,
                reasons = listOf("Scene geometry untrusted: optical flow erratic or unverified"),
                timestampMs = nowMs,
            )
        }

        // 5. World Model Staleness
        if (worldModel.isStale(nowMs)) {
            return SafetyGateVerdict(
                safeCommand = MotionCommand.STOP.copy(
                    source = "MAP_STALE",
                    timestampNs = nowMs * 1_000_000L,
                ),
                action = SafetyGateAction.STOP,
                reasons = listOf("Perception / map evidence is stale (>1000ms old)"),
                timestampMs = nowMs,
            )
        }

        // 6. Caution Zone: Clamp Forward Velocity
        if (safetyState.level == SafetyLevel.CAUTION) {
            val clampedLinear = proposed.linearVelocityMps.coerceIn(-safetyState.speedLimitMps, safetyState.speedLimitMps)
            return SafetyGateVerdict(
                safeCommand = proposed.copy(
                    linearVelocityMps = clampedLinear,
                    timestampNs = nowMs * 1_000_000L,
                ),
                action = SafetyGateAction.SLOW,
                reasons = safetyState.reasons,
                timestampMs = nowMs,
            )
        }

        // 7. Nominal Clear
        return SafetyGateVerdict(
            safeCommand = proposed.copy(timestampNs = nowMs * 1_000_000L),
            action = SafetyGateAction.ALLOW,
            reasons = emptyList(),
            timestampMs = nowMs,
        )
    }
}
