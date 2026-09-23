package com.minesafety.roboeye.core.model

/**
 * Deterministic safety clearance levels.
 */
enum class SafetyLevel {
    /** All safety metrics well within green thresholds. Full speed permitted. */
    NOMINAL,
    /** Approaching obstacle boundary, moderate tilt, or low visibility. Speed reduced. */
    CAUTION,
    /** Critical obstacle proximity (<0.5m), extreme tilt (>25°), or lost transport heartbeat. Full halt. */
    EMERGENCY_STOP,
}

/**
 * Evaluated safety verdict from the deterministic SafetyController.
 */
data class SafetyState(
    val level: SafetyLevel = SafetyLevel.NOMINAL,
    val isEmergencyStop: Boolean = false,
    val speedLimitMps: Float = 1.0f,
    val reasons: List<String> = emptyList(),
    val minObstacleDistanceM: Float? = null,
    val evaluatedAtMs: Long = System.currentTimeMillis(),
) {
    companion object {
        val NOMINAL = SafetyState(
            level = SafetyLevel.NOMINAL,
            isEmergencyStop = false,
            speedLimitMps = 1.0f,
            reasons = emptyList(),
        )
    }
}
