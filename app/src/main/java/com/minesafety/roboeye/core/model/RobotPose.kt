package com.minesafety.roboeye.core.model

/**
 * 2D / 3D spatial pose of the robot in the local odometry or map coordinate frame.
 */
data class RobotPose(
    val xM: Float = 0.0f,
    val yM: Float = 0.0f,
    val zM: Float = 0.0f,
    /** Heading / Yaw angle in degrees (-180° to 180°). */
    val yawDeg: Float = 0.0f,
    val pitchDeg: Float = 0.0f,
    val rollDeg: Float = 0.0f,
    /** Pose estimation confidence score (0.0 to 1.0). */
    val confidence: Float = 1.0f,
    val frameId: String = "odom",
    val timestampNs: Long = System.nanoTime(),
)
