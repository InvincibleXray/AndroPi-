package com.minesafety.roboeye.sensors.location

import com.minesafety.roboeye.core.LocationReading
import com.minesafety.roboeye.sensors.base.SensorProvider

/**
 * Interface for GNSS / location providers.
 */
interface LocationProvider : SensorProvider<LocationReading> {
    /** Checks whether location permissions are granted. */
    fun hasPermission(): Boolean
}
