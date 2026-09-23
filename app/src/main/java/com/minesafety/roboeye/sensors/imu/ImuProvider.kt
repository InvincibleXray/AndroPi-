package com.minesafety.roboeye.sensors.imu

import com.minesafety.roboeye.core.ImuReading
import com.minesafety.roboeye.sensors.base.SensorProvider

/**
 * Interface for Inertial Measurement Unit providers.
 */
interface ImuProvider : SensorProvider<ImuReading> {
    /** Whether hardware accelerometer is present. */
    val hasAccelerometer: Boolean

    /** Whether fused linear acceleration is available in hardware. */
    val hasLinearAcceleration: Boolean

    /** Whether hardware gyroscope is present. */
    val hasGyroscope: Boolean

    /** Sets mount pitch and roll offsets to calibrate zero level on the chassis. */
    fun setMountOffsets(pitchDeg: Float, rollDeg: Float)

    /** Calibrates current orientation as chassis level. Returns Pair(pitchOffset, rollOffset) or null. */
    fun calibrateLevel(): Pair<Float, Float>?
}
