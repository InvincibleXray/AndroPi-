package com.minesafety.roboeye.navigation

import com.minesafety.roboeye.core.model.MotionCommand
import com.minesafety.roboeye.core.model.RobotPose
import com.minesafety.roboeye.fusion.FusedWorldState
import kotlinx.coroutines.flow.StateFlow

data class NavGoal(val targetX: Float, val targetY: Float, val toleranceRadiusM: Float = 0.5f)

/**
 * Autonomous Navigation and Path Planning boundary interface.
 */
interface NavigationEngine {
    /** Whether navigation is actively pursuing a waypoint goal. */
    val isNavigating: Boolean

    /** Sets a navigation target in map frame. */
    fun setGoal(goal: NavGoal)

    /** Cancels the current navigation goal and halts planning. */
    fun cancelGoal()

    /** Computes the next motion command given current pose and fused world state. */
    fun computeVelocity(currentPose: RobotPose, world: FusedWorldState): MotionCommand
}
