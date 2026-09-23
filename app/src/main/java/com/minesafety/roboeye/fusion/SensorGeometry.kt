package com.minesafety.roboeye.fusion

import kotlin.math.cos
import kotlin.math.sin

/**
 * Standard robotics coordinate convention for Rover Brain:
 * - +X: Forward (meters)
 * - +Y: Left (meters)
 * - +Z: Up (meters)
 * - Yaw: Counter-clockwise positive (0° = Forward, +90° = Left, -90° = Right)
 */

enum class CorridorSector {
    LEFT,
    CENTER,
    RIGHT,
}

data class Point2D(
    val xM: Float,
    val yM: Float,
)

data class RoverFootprint(
    val widthM: Float = 0.35f,
    val lengthM: Float = 0.45f,
    val safetyMarginM: Float = 0.15f,
) {
    val totalHalfWidthM: Float get() = (widthM / 2.0f) + safetyMarginM
    val totalLengthM: Float get() = lengthM + safetyMarginM

    fun classifySector(point: Point2D): CorridorSector = when {
        point.yM > (widthM / 2.0f) -> CorridorSector.LEFT
        point.yM < -(widthM / 2.0f) -> CorridorSector.RIGHT
        else -> CorridorSector.CENTER
    }

    fun isWithinFootprint(point: Point2D): Boolean {
        return point.xM in -safetyMarginM..totalLengthM &&
                point.yM in -totalHalfWidthM..totalHalfWidthM
    }
}

/**
 * Mounting location and orientation of a physical distance sensor relative to robot center.
 */
data class DistanceSensorMount(
    val sensorId: String,
    val xM: Float,
    val yM: Float,
    val zM: Float = 0.0f,
    val yawDeg: Float,
    val pitchDeg: Float = 0.0f,
    val rangeMinM: Float = 0.02f,
    val rangeMaxM: Float = 4.5f,
) {
    /**
     * Projects a measured radial distance along the sensor heading into rover base-frame coordinates.
     */
    fun project(distanceM: Float): Point2D {
        val yawRad = Math.toRadians(yawDeg.toDouble())
        val x = xM + distanceM * cos(yawRad).toFloat()
        val y = yM + distanceM * sin(yawRad).toFloat()
        return Point2D(x, y)
    }

    companion object {
        val DEFAULT_FRONT = DistanceSensorMount(
            sensorId = "ULTRASONIC_FRONT",
            xM = 0.225f,
            yM = 0.0f,
            yawDeg = 0.0f,
        )
        val DEFAULT_LEFT = DistanceSensorMount(
            sensorId = "ULTRASONIC_LEFT",
            xM = 0.150f,
            yM = 0.175f,
            yawDeg = 45.0f,
        )
        val DEFAULT_RIGHT = DistanceSensorMount(
            sensorId = "ULTRASONIC_RIGHT",
            xM = 0.150f,
            yM = -0.175f,
            yawDeg = -45.0f,
        )
        val DEFAULT_TOF = DistanceSensorMount(
            sensorId = "TOF_FRONT",
            xM = 0.230f,
            yM = 0.0f,
            yawDeg = 0.0f,
            rangeMinM = 0.03f,
            rangeMaxM = 2.0f,
        )
    }
}
