package com.minesafety.roboeye.core.time

/**
 * High-precision monotonic clock abstraction for robotics timing loops and kinematics.
 */
interface RobotClock {
    /** Returns current monotonic time in nanoseconds. Immune to wall-clock adjustments. */
    fun nowNs(): Long

    /** Returns current monotonic time in milliseconds. */
    fun nowMs(): Long = nowNs() / 1_000_000L

    companion object {
        val SYSTEM: RobotClock = object : RobotClock {
            override fun nowNs(): Long = System.nanoTime()
        }
    }
}
