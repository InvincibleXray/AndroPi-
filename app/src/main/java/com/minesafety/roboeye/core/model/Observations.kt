package com.minesafety.roboeye.core.model

/**
 * Sensor-derived obstacle observation relative to robot base frame.
 */
data class ObstacleObservation(
    val id: String,
    /** Radial distance in meters from the robot center. */
    val distanceM: Float,
    /** Bearing / azimuth angle in degrees relative to the vehicle heading (-180° to +180°). */
    val bearingDeg: Float,
    /** Estimated real-world width in meters. */
    val widthM: Float? = null,
    /** Estimated Time-To-Collision in seconds. Negative or null if diverging. */
    val timeToCollisionSec: Float? = null,
    /** Provenance sensor source (e.g. "ESP32_ULTRASONIC", "OPTICAL_FLOW", "VISION_GEOMETRY"). */
    val source: String,
    val confidence: Float = 1.0f,
    val timestampMs: Long = System.currentTimeMillis(),
)

/**
 * Free-space corridor observation describing traversable ground geometry.
 */
data class FreeSpaceObservation(
    val isSupported: Boolean = false,
    /** Clear forward distance in meters along the central travel corridor. */
    val clearDistanceAheadM: Float = 0.0f,
    val traversableWidthLeftM: Float = 0.0f,
    val traversableWidthRightM: Float = 0.0f,
    val isPassable: Boolean = true,
    val confidence: Float = 0.0f,
    val timestampMs: Long = System.currentTimeMillis(),
)
