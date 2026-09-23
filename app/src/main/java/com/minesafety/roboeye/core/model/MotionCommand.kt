package com.minesafety.roboeye.core.model

/**
 * Immutable command to drive the rover chassis.
 *
 * Neither UI components nor AI algorithms can send this directly to hardware;
 * all commands must pass through the deterministic SafetyController.
 */
data class MotionCommand(
    /** Desired forward linear velocity in meters per second (-1.0 to 1.0 m/s). */
    val linearVelocityMps: Float = 0.0f,
    /** Desired angular velocity / yaw rate in radians per second (-2.0 to 2.0 rad/s). */
    val angularVelocityRadS: Float = 0.0f,
    /** Immediate emergency stop trigger. When true, motor output is cut instantly. */
    val emergencyStop: Boolean = false,
    /** Monotonic timestamp when the command was generated. */
    val timestampNs: Long = System.nanoTime(),
    /** Source that generated this command (e.g. "AUTONOMOUS_NAV", "TELEOP", "SAFETY_ESTOP"). */
    val source: String = "IDLE",
) {
    companion object {
        val STOP = MotionCommand(0.0f, 0.0f, false, source = "STOP")
        val EMERGENCY_STOP = MotionCommand(0.0f, 0.0f, true, source = "EMERGENCY_STOP")
    }
}
