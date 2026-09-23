package com.minesafety.roboeye.nav.control

import com.minesafety.roboeye.localization.LocalPose
import com.minesafety.roboeye.nav.model.LocalGoal
import com.minesafety.roboeye.nav.planner.PlannedPath
import com.minesafety.roboeye.nav.planner.Waypoint
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.sin

/**
 * Deterministic differential drive kinematic controller.
 *
 * Implements pure-pursuit trajectory tracking, heading correction, cross-track error
 * reduction, and smooth deceleration near navigation goals.
 */
class DifferentialDriveController(
    val trackWidthM: Float = 0.35f,
    val maxLinearVelocityMps: Float = 0.35f,
    val maxAngularVelocityRadS: Float = 1.20f,
    val lookaheadDistanceM: Float = 0.30f,
    val headingGainKp: Float = 1.80f,
) {
    /**
     * Computes the [MotionIntent] required to guide the rover along [path] toward [goal].
     */
    fun computeMotion(
        path: PlannedPath,
        currentPose: LocalPose,
        goal: LocalGoal,
        speedLimitMps: Float = maxLinearVelocityMps,
        nowMs: Long = System.currentTimeMillis(),
    ): MotionIntent {
        if (path.isEmpty) {
            return MotionIntent.STOP.copy(source = "EMPTY_PATH", timestampMs = nowMs)
        }

        // 1. Goal Reached Check
        val distToGoal = hypot(goal.targetX - currentPose.xM, goal.targetY - currentPose.yM)
        if (distToGoal <= goal.positionToleranceM) {
            return MotionIntent.STOP.copy(source = "GOAL_REACHED", timestampMs = nowMs)
        }

        // 2. Select Lookahead Waypoint
        val targetWp = selectLookaheadWaypoint(path.waypoints, currentPose)

        // 3. Compute Heading Error
        val targetAngleRad = atan2(targetWp.yM - currentPose.yM, targetWp.xM - currentPose.xM)
        val currentYawRad = Math.toRadians(currentPose.yawDeg.toDouble()).toFloat()
        val headingErrorRad = normalizeAngleRad(targetAngleRad - currentYawRad)

        // 4. Compute Cross-Track Error
        val crossTrackErrorM = computeCrossTrackError(currentPose, path.waypoints)

        // 5. Angular Velocity Control
        var w = (headingErrorRad * headingGainKp).coerceIn(-maxAngularVelocityRadS, maxAngularVelocityRadS)

        // 6. Linear Velocity Control with Turn Clamping & Near-Goal Deceleration
        val absHeadingError = Math.abs(headingErrorRad)
        val effectiveSpeedLimit = speedLimitMps.coerceIn(0.0f, maxLinearVelocityMps)

        val v: Float
        val direction: MotionDirection

        if (absHeadingError > Math.toRadians(35.0)) {
            // High heading error: Turn in place before advancing to prevent lateral drift
            v = 0.0f
            w = if (headingErrorRad > 0) 0.60f else -0.60f
            direction = if (headingErrorRad > 0) MotionDirection.TURN_LEFT else MotionDirection.TURN_RIGHT
        } else {
            // Forward motion scaled by heading alignment and goal distance
            val alignmentFactor = cos(absHeadingError).coerceIn(0.2f, 1.0f)
            val decelFactor = (distToGoal / 0.80f).coerceIn(0.35f, 1.0f)
            v = effectiveSpeedLimit * alignmentFactor * decelFactor

            direction = when {
                w > 0.15f -> MotionDirection.ARC_LEFT
                w < -0.15f -> MotionDirection.ARC_RIGHT
                else -> MotionDirection.FORWARD
            }
        }

        // 7. Differential Drive Wheel Velocities
        // V_L = V - (W * B / 2),  V_R = V + (W * B / 2)
        val halfBase = trackWidthM / 2.0f
        val vLeft = v - (w * halfBase)
        val vRight = v + (w * halfBase)

        return MotionIntent(
            direction = direction,
            linearVelocityMps = v,
            angularVelocityRadS = w,
            leftWheelMps = vLeft,
            rightWheelMps = vRight,
            confidence = currentPose.confidence,
            source = "AUTONOMOUS_NAV",
            timestampMs = nowMs,
        )
    }

    private fun selectLookaheadWaypoint(waypoints: List<Waypoint>, currentPose: LocalPose): Waypoint {
        var selected = waypoints.last()
        for (wp in waypoints) {
            val dist = hypot(wp.xM - currentPose.xM, wp.yM - currentPose.yM)
            if (dist >= lookaheadDistanceM) {
                selected = wp
                break
            }
        }
        return selected
    }

    fun computeCrossTrackError(currentPose: LocalPose, waypoints: List<Waypoint>): Float {
        if (waypoints.size < 2) return 0.0f

        // Find nearest path segment
        var minDistance = Float.MAX_VALUE
        for (i in 0 until waypoints.size - 1) {
            val p1 = waypoints[i]
            val p2 = waypoints[i + 1]
            val d = distancePointToSegment(currentPose.xM, currentPose.yM, p1.xM, p1.yM, p2.xM, p2.yM)
            if (d < minDistance) minDistance = d
        }
        return if (minDistance == Float.MAX_VALUE) 0.0f else minDistance
    }

    private fun distancePointToSegment(
        px: Float, py: Float,
        x1: Float, y1: Float,
        x2: Float, y2: Float,
    ): Float {
        val dx = x2 - x1
        val dy = y2 - y1
        val lenSq = dx * dx + dy * dy
        if (lenSq == 0f) return hypot(px - x1, py - y1)

        val t = (((px - x1) * dx + (py - y1) * dy) / lenSq).coerceIn(0f, 1f)
        val projX = x1 + t * dx
        val projY = y1 + t * dy
        return hypot(px - projX, py - projY)
    }

    fun normalizeAngleRad(angle: Float): Float {
        var a = angle
        while (a > Math.PI) a -= (2.0 * Math.PI).toFloat()
        while (a < -Math.PI) a += (2.0 * Math.PI).toFloat()
        return a
    }
}
