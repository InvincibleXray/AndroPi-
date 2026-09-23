package com.minesafety.roboeye.core.model

import com.minesafety.roboeye.core.ImuReading
import com.minesafety.roboeye.core.LocationReading

/**
 * Dynamic sensor availability status.
 *
 * Never fabricate sensor values: if hardware is absent or permission is denied,
 * state reports UNAVAILABLE or NO_PERMISSION, and runtime readings remain null.
 */
enum class SensorStatus {
    ACTIVE,
    WAITING,
    UNAVAILABLE,
    NO_PERMISSION,
}

/**
 * Ambient light sensor reading (illuminance in lux).
 */
data class LightReading(
    val lux: Float,
    val timestampNs: Long = System.nanoTime(),
)

/**
 * Calibrated magnetometer reading (magnetic field in micro-Tesla µT).
 */
data class MagnetometerReading(
    val mxUt: Float,
    val myUt: Float,
    val mzUt: Float,
    val azimuthDeg: Float? = null,
    val timestampNs: Long = System.nanoTime(),
)

/**
 * Synthetic / fused rotation vector reading (unit quaternion components).
 */
data class RotationVectorReading(
    val qx: Float,
    val qy: Float,
    val qz: Float,
    val qw: Float,
    val headingDeg: Float? = null,
    val timestampNs: Long = System.nanoTime(),
)

/**
 * Aggregated state of all phone-internal sensors.
 */
data class PhoneSensorState(
    val imuStatus: SensorStatus = SensorStatus.WAITING,
    val imu: ImuReading? = null,
    val lightStatus: SensorStatus = SensorStatus.UNAVAILABLE,
    val light: LightReading? = null,
    val magnetometerStatus: SensorStatus = SensorStatus.UNAVAILABLE,
    val magnetometer: MagnetometerReading? = null,
    val rotationStatus: SensorStatus = SensorStatus.UNAVAILABLE,
    val rotation: RotationVectorReading? = null,
    val locationStatus: SensorStatus = SensorStatus.WAITING,
    val location: LocationReading? = null,
    val batteryPercent: Int? = null,
    val batteryTempC: Float? = null,
    val isCharging: Boolean = false,
    val timestampMs: Long = System.currentTimeMillis(),
)

/**
 * External sensor state reported by ESP32 chassis over RobotTransport.
 */
data class Esp32SensorState(
    val ultrasonicFrontM: Float? = null,
    val ultrasonicLeftM: Float? = null,
    val ultrasonicRightM: Float? = null,
    val tofFrontM: Float? = null,
    val irLeft: Int? = null,
    val irCenter: Int? = null,
    val irRight: Int? = null,
    val wheelTicksLeft: Long? = null,
    val wheelTicksRight: Long? = null,
    val batteryVoltageV: Float? = null,
    val chassisImuPitchDeg: Float? = null,
    val chassisImuRollDeg: Float? = null,
    val hardwareEStop: Boolean = false,
    val uptimeMs: Long? = null,
    val timestampMs: Long = System.currentTimeMillis(),
) {
    val minObstacleM: Float?
        get() = listOfNotNull(ultrasonicFrontM, ultrasonicLeftM, ultrasonicRightM, tofFrontM).minOrNull()
}
