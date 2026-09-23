package com.minesafety.roboeye.localization

import com.minesafety.roboeye.core.model.RobotPose
import kotlinx.coroutines.flow.StateFlow

/**
 * Robot localization & odometry estimation boundary interface.
 */
interface LocalizationProvider {
    /** Current estimated robot pose in world coordinates. */
    val currentPose: StateFlow<RobotPose>

    /** Resets the odometry origin to the current robot position. */
    fun resetOrigin()
}
