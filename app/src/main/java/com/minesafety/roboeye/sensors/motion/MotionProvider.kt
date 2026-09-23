package com.minesafety.roboeye.sensors.motion

import com.minesafety.roboeye.sensors.base.SensorProvider

data class MotionState(
    val isMoving: Boolean = false,
    val accelerationMagnitudeG: Float = 0.0f,
    val timestampNs: Long = System.nanoTime(),
)

/**
 * Interface for phone motion and vibration detection providers.
 */
interface MotionProvider : SensorProvider<MotionState>
