package com.minesafety.roboeye.robot.mode

/**
 * High-level robot operational modes.
 */
enum class RobotMode(val modeId: Int, val label: String, val allowsAutonomousMotion: Boolean) {
    /** MODE 1 — Manual Teleoperation (direct human control, safety limits active). */
    MANUAL(1, "MODE 1 — Manual Teleop", false),

    /** MODE 2 — Autonomous Navigation (local obstacle avoidance, goal point driving). */
    AUTONOMOUS_NAVIGATION(2, "MODE 2 — Autonomous Navigation", true),

    /** MODE 3 — Follow Target (track and maintain distance from identified object). */
    FOLLOW_TARGET(3, "MODE 3 — Follow Target", true),

    /** MODE 4 — Path Follow (drive predefined waypoint corridor). */
    PATH_FOLLOW(4, "MODE 4 — Path Follow", true),

    /** MODE 5 — Search / Explore (frontier exploration in unmapped space). */
    SEARCH_EXPLORE(5, "MODE 5 — Search / Explore", true),
}
