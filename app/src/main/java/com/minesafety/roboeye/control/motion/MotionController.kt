package com.minesafety.roboeye.control.motion

import com.minesafety.roboeye.core.model.MotionCommand
import kotlinx.coroutines.flow.StateFlow

/**
 * Interface for motion controllers translating navigation intents or manual teleop inputs
 * into candidate [MotionCommand] structures.
 *
 * NOTE: Output commands MUST always pass through [com.minesafety.roboeye.control.safety.SafetyController]
 * before reaching the motor transport.
 */
interface MotionController {
    /** Latest candidate motion command proposed by the controller. */
    val proposedCommand: StateFlow<MotionCommand>

    /** Proposes a manual or autonomous motion command. */
    fun proposeMotion(linearVelocityMps: Float, angularVelocityRadS: Float, source: String = "MANUAL")

    /** Proposes an immediate stop. */
    fun proposeStop(source: String = "STOP")
}
