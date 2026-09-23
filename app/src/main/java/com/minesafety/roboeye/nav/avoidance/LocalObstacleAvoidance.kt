package com.minesafety.roboeye.nav.avoidance

import com.minesafety.roboeye.nav.model.NavigationWorldModel
import com.minesafety.roboeye.nav.planner.PathValidator
import com.minesafety.roboeye.nav.planner.PlannedPath
import com.minesafety.roboeye.vision.GeometryTrustLevel
import com.minesafety.roboeye.vision.RecommendedCorridor

/**
 * Tactical avoidance action produced by the local obstacle avoidance assessor.
 */
sealed interface AvoidanceAction {
    data object Clear : AvoidanceAction
    data class SlowDown(val reason: String, val speedLimitMps: Float = 0.20f) : AvoidanceAction
    data class Stop(val reason: String) : AvoidanceAction
    data class EmergencyBrake(val reason: String) : AvoidanceAction
    data class ReplanRequired(val reason: String) : AvoidanceAction
}

/**
 * Local obstacle avoidance engine continuously monitoring visual corridors, TTC, and geometry trust.
 */
class LocalObstacleAvoidance(
    private val pathValidator: PathValidator = PathValidator(),
) {
    companion object {
        const val CRITICAL_TTC_SEC = 1.20f
        const val CAUTION_TTC_SEC = 2.50f
    }

    /**
     * Evaluates tactical visual collision risk using perception and path evidence.
     */
    fun evaluate(
        worldModel: NavigationWorldModel,
        currentPath: PlannedPath? = null,
        nowMs: Long = System.currentTimeMillis(),
    ): AvoidanceAction {
        // 1. Scene Geometry Trust Gate
        when (worldModel.geometryTrust) {
            GeometryTrustLevel.UNTRUSTED -> {
                return AvoidanceAction.Stop("Scene geometry untrusted: unknown or erratic visual flow")
            }
            GeometryTrustLevel.DEGRADED -> {
                // Caution: slow down rover in degraded visual conditions
            }
            GeometryTrustLevel.TRUSTED -> Unit
        }

        // 2. Optical Flow Time-To-Collision (TTC) Gate
        val minTtc = worldModel.ttcResult?.minTtcSec
        if (minTtc != null) {
            if (minTtc <= CRITICAL_TTC_SEC) {
                return AvoidanceAction.EmergencyBrake(
                    "Critical TTC collision danger: ${"%.2f".format(minTtc)}s <= $CRITICAL_TTC_SEC s",
                )
            }
            if (minTtc <= CAUTION_TTC_SEC) {
                return AvoidanceAction.SlowDown(
                    "Caution TTC obstacle proximity: ${"%.2f".format(minTtc)}s <= $CAUTION_TTC_SEC s",
                    speedLimitMps = 0.15f,
                )
            }
        }

        // 3. Recommended Corridor Evaluation
        if (worldModel.recommendedCorridor == RecommendedCorridor.NONE_AVAILABLE) {
            return AvoidanceAction.Stop("All visual corridors (Left/Center/Right) obstructed")
        }

        // 4. Path Obstruction Validation
        if (currentPath != null && !currentPath.isEmpty) {
            val validation = pathValidator.validatePath(currentPath, worldModel, nowMs)
            if (!validation.isValid) {
                return AvoidanceAction.ReplanRequired(
                    validation.reason ?: "Planned path obstructed by dynamic obstacle",
                )
            }
        }

        // 5. Degraded geometry speed clamp
        if (worldModel.geometryTrust == GeometryTrustLevel.DEGRADED) {
            return AvoidanceAction.SlowDown(
                "Degraded geometry trust; speed clamped for safety",
                speedLimitMps = 0.20f,
            )
        }

        return AvoidanceAction.Clear
    }
}
