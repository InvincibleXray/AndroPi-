package com.minesafety.roboeye.robot.mode

import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow

/**
 * Central robot operational mode coordinator.
 *
 * Prevents mode transitions when safety guards or hardware links are not in a healthy state.
 */
class ModeManager(initialMode: RobotMode = RobotMode.MANUAL) {

    private val _currentMode = MutableStateFlow(initialMode)
    val currentMode: StateFlow<RobotMode> = _currentMode.asStateFlow()

    /** Callback invoked when mode changes, enabling controllers to halt motors or reset planners. */
    var onModeTransition: ((oldMode: RobotMode, newMode: RobotMode) -> Unit)? = null

    fun requestMode(newMode: RobotMode, isEStopActive: Boolean = false): Boolean {
        if (isEStopActive && newMode.allowsAutonomousMotion) {
            return false // Cannot enter autonomous modes while under active E-Stop
        }
        val oldMode = _currentMode.value
        if (oldMode == newMode) return true

        _currentMode.value = newMode
        onModeTransition?.invoke(oldMode, newMode)
        return true
    }

    fun forceManual() {
        requestMode(RobotMode.MANUAL, isEStopActive = false)
    }
}
