package com.minesafety.roboeye.nav.model

import com.minesafety.roboeye.mapping.MapCellState
import kotlin.math.hypot

/**
 * Local 2D navigation goal defined in the rover's local spatial frame (meters).
 */
data class LocalGoal(
    val targetX: Float,
    val targetY: Float,
    val headingToleranceRad: Float? = null,
    val positionToleranceM: Float = 0.20f,
    val timestampMs: Long = System.currentTimeMillis(),
    val valid: Boolean = true,
) {
    fun distanceTo(x: Float, y: Float): Float {
        return hypot(targetX - x, targetY - y)
    }

    companion object {
        val NONE = LocalGoal(0.0f, 0.0f, valid = false)
    }
}

/**
 * Result of goal validation against current world model.
 */
data class GoalValidationResult(
    val isValid: Boolean,
    val reason: String? = null,
)

/**
 * Strict validator verifying candidate navigation goals against map bounds, occupancy,
 * and localization status before planning.
 */
object GoalValidator {

    const val MIN_DISTANCE_M = 0.10f
    const val MAX_DISTANCE_M = 4.50f

    fun validate(goal: LocalGoal, worldModel: NavigationWorldModel): GoalValidationResult {
        if (!goal.valid) {
            return GoalValidationResult(false, "Goal is explicitly marked invalid")
        }

        // 1. Localization health check
        if (!worldModel.isLocalizationValid) {
            return GoalValidationResult(false, "Localization tracking is lost or degraded (${worldModel.pose.trackingState})")
        }

        // 2. Map bounds check
        val map = worldModel.spatialMap
        val gx = ((goal.targetX / map.resolutionM) + map.gridWidth / 2f).toInt()
        val gy = ((goal.targetY / map.resolutionM) + map.gridHeight / 2f).toInt()

        if (gx !in 0 until map.gridWidth || gy !in 0 until map.gridHeight) {
            return GoalValidationResult(false, "Goal ($gx, $gy) lies outside supported 6m x 6m local grid")
        }

        // 3. Goal distance envelope
        val currentPose = worldModel.pose
        val distance = goal.distanceTo(currentPose.xM, currentPose.yM)
        if (distance < MIN_DISTANCE_M) {
            return GoalValidationResult(false, "Goal is already reached or too close ($distance m < $MIN_DISTANCE_M m)")
        }
        if (distance > MAX_DISTANCE_M) {
            return GoalValidationResult(false, "Goal distance exceeds supported local planning horizon ($distance m > $MAX_DISTANCE_M m)")
        }

        // 4. "UNKNOWN IS NOT SAFE" & Obstacle clearance check
        val targetCell = map.cells[gx][gy]
        if (targetCell.state == MapCellState.OCCUPIED) {
            return GoalValidationResult(false, "Goal lies within an occupied obstacle cell")
        }
        if (targetCell.state == MapCellState.UNCERTAIN) {
            return GoalValidationResult(false, "Goal lies within an unknown/uncertain cell: UNKNOWN IS NOT SAFE")
        }

        return GoalValidationResult(true)
    }
}
