package com.minesafety.roboeye.sensors.base

import com.minesafety.roboeye.core.model.SensorStatus
import kotlinx.coroutines.flow.StateFlow

/**
 * Universal contract for an onboard sensor provider.
 *
 * Dynamic detection is mandatory: if hardware is absent or permission denied,
 * [status] must reflect UNAVAILABLE or NO_PERMISSION.
 * Real sensor values are never fabricated.
 */
interface SensorProvider<T> {
    /** Human-readable provider name (e.g. "Phone IMU", "Ambient Light Sensor"). */
    val name: String

    /** Dynamic operational status. */
    val status: StateFlow<SensorStatus>

    /** Latest sensor reading, or null if the sensor is unavailable, waiting, or stopped. */
    val reading: StateFlow<T?>

    /** Starts sampling. */
    fun start()

    /** Stops sampling to conserve battery and CPU. */
    fun stop()
}
