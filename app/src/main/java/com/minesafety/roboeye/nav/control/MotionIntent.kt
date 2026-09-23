package com.minesafety.roboeye.nav.control

import com.minesafety.roboeye.core.model.MotionCommand

/**
 * Directional category of rover locomotion.
 */
enum class MotionDirection {
    STOP,
    FORWARD,
    REVERSE,
    TURN_LEFT,
    TURN_RIGHT,
    ARC_LEFT,
    ARC_RIGHT,
}

/**
 * High-level motion intent produced by the differential drive controller.
 * Contains both body-frame velocities (V, W) and individual wheel linear speeds (V_L, V_R).
 */
data class MotionIntent(
    val direction: MotionDirection = MotionDirection.STOP,
    val linearVelocityMps: Float = 0.0f,
    val angularVelocityRadS: Float = 0.0f,
    val leftWheelMps: Float = 0.0f,
    val rightWheelMps: Float = 0.0f,
    val confidence: Float = 1.0f,
    val source: String = "NAV_CONTROLLER",
    val timestampMs: Long = System.currentTimeMillis(),
) {
    fun toMotionCommand(): MotionCommand {
        val isEstop = direction == MotionDirection.STOP &&
                source.contains("ESTOP", ignoreCase = true)
        return MotionCommand(
            linearVelocityMps = linearVelocityMps,
            angularVelocityRadS = angularVelocityRadS,
            emergencyStop = isEstop,
            source = source,
        )
    }

    companion object {
        val STOP = MotionIntent(
            direction = MotionDirection.STOP,
            linearVelocityMps = 0.0f,
            angularVelocityRadS = 0.0f,
            leftWheelMps = 0.0f,
            rightWheelMps = 0.0f,
            confidence = 1.0f,
            source = "STOP",
        )
    }
}
