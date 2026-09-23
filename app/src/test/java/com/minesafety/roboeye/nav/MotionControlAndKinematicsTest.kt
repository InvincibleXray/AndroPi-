package com.minesafety.roboeye.nav

import com.minesafety.roboeye.localization.LocalPose
import com.minesafety.roboeye.localization.TrackingQuality
import com.minesafety.roboeye.nav.control.DifferentialDriveController
import com.minesafety.roboeye.nav.control.MotionDirection
import com.minesafety.roboeye.nav.model.LocalGoal
import com.minesafety.roboeye.nav.planner.PlannedPath
import com.minesafety.roboeye.nav.planner.Waypoint
import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Before
import org.junit.Test

/**
 * Deterministic unit tests covering:
 * I. Heading error calculation & angle normalization
 * J. Cross-track error
 * K. Motion intent generation & differential drive kinematic wheel speeds
 * L. Speed limiting under turn curvature and caution
 */
class MotionControlAndKinematicsTest {

    private lateinit var controller: DifferentialDriveController

    @Before
    fun setUp() {
        controller = DifferentialDriveController(
            trackWidthM = 0.35f,
            maxLinearVelocityMps = 0.35f,
            maxAngularVelocityRadS = 1.20f,
            lookaheadDistanceM = 0.30f,
            headingGainKp = 1.80f,
        )
    }

    // --- I. HEADING ERROR TESTS ---
    @Test
    fun testHeadingError_NormalizesAnglesCorrectly() {
        // Angle normalization wraps into [-PI, PI]
        assertEquals(0.0f, controller.normalizeAngleRad(0.0f), 0.001f)
        assertEquals(Math.PI.toFloat(), Math.abs(controller.normalizeAngleRad(Math.PI.toFloat())), 0.001f)

        // 3PI/2 (-90 deg) should normalize to -PI/2
        val overPi = (1.5 * Math.PI).toFloat()
        assertEquals((-0.5 * Math.PI).toFloat(), controller.normalizeAngleRad(overPi), 0.001f)

        // -3PI/2 (+90 deg) should normalize to +PI/2
        val underMinusPi = (-1.5 * Math.PI).toFloat()
        assertEquals((0.5 * Math.PI).toFloat(), controller.normalizeAngleRad(underMinusPi), 0.001f)
    }

    // --- J. CROSS-TRACK ERROR TESTS ---
    @Test
    fun testCrossTrackError_CalculatesZeroOnStraightSegment() {
        val path = listOf(
            Waypoint(0.0f, 0.0f),
            Waypoint(1.0f, 0.0f),
            Waypoint(2.0f, 0.0f),
        )
        // Rover directly on path centerline at (0.5m, 0.0m)
        val poseOnCenterline = LocalPose(xM = 0.5f, yM = 0.0f, yawDeg = 0.0f)
        val error = controller.computeCrossTrackError(poseOnCenterline, path)
        assertEquals(0.0f, error, 0.001f)

        // Rover displaced laterally to the left by +0.15m at (0.5m, 0.15m)
        val poseDisplaced = LocalPose(xM = 0.5f, yM = 0.15f, yawDeg = 0.0f)
        val errorDisplaced = controller.computeCrossTrackError(poseDisplaced, path)
        assertEquals(0.15f, errorDisplaced, 0.005f)
    }

    // --- K. MOTION INTENT & DIFFERENTIAL KINEMATICS TESTS ---
    @Test
    fun testDifferentialDrive_ComputesCorrectWheelVelocities() {
        // Path straight ahead (+X)
        val path = PlannedPath(
            waypoints = listOf(
                Waypoint(0.0f, 0.0f),
                Waypoint(0.5f, 0.0f),
                Waypoint(1.5f, 0.0f),
            ),
            totalLengthM = 1.5f,
        )
        val pose = LocalPose(xM = 0.0f, yM = 0.0f, yawDeg = 0.0f, confidence = 0.95f, trackingState = TrackingQuality.TRACKING)
        val goal = LocalGoal(targetX = 1.5f, targetY = 0.0f)

        val intent = controller.computeMotion(path, pose, goal)

        assertEquals(MotionDirection.FORWARD, intent.direction)
        assertTrue("Linear velocity should be positive", intent.linearVelocityMps > 0f)
        assertEquals("Angular velocity should be ~0 on straight path", 0.0f, intent.angularVelocityRadS, 0.05f)

        // Differential wheel kinematics: V_L = V - W*B/2, V_R = V + W*B/2
        // On straight forward motion, V_L == V_R == V
        assertEquals(intent.linearVelocityMps, intent.leftWheelMps, 0.01f)
        assertEquals(intent.linearVelocityMps, intent.rightWheelMps, 0.01f)
    }

    @Test
    fun testDifferentialDrive_TurnsInPlaceOnLargeHeadingError() {
        // Path is 90 degrees to the left (+Y), but rover is pointing forward (Yaw = 0 deg)
        val path = PlannedPath(
            waypoints = listOf(
                Waypoint(0.0f, 0.0f),
                Waypoint(0.0f, 1.0f),
            ),
            totalLengthM = 1.0f,
        )
        val pose = LocalPose(xM = 0.0f, yM = 0.0f, yawDeg = 0.0f)
        val goal = LocalGoal(targetX = 0.0f, targetY = 1.0f)

        val intent = controller.computeMotion(path, pose, goal)

        // High heading error (> 35 deg): Must command turn-in-place with zero forward translation
        assertEquals(MotionDirection.TURN_LEFT, intent.direction)
        assertEquals(0.0f, intent.linearVelocityMps, 0.001f)
        assertTrue("Turn rate should be positive (CCW)", intent.angularVelocityRadS > 0f)

        // In place CCW turn: Left wheel goes backward, right wheel goes forward
        assertTrue("Left wheel should be negative in CCW turn", intent.leftWheelMps < 0f)
        assertTrue("Right wheel should be positive in CCW turn", intent.rightWheelMps > 0f)
    }

    // --- L. SPEED LIMITING & DECELERATION TESTS ---
    @Test
    fun testSpeedLimiting_ClampsVelocityUnderCautionConstraint() {
        val path = PlannedPath(
            waypoints = listOf(
                Waypoint(0.0f, 0.0f),
                Waypoint(1.0f, 0.0f),
            ),
            totalLengthM = 1.0f,
        )
        val pose = LocalPose(xM = 0.0f, yM = 0.0f, yawDeg = 0.0f)
        val goal = LocalGoal(targetX = 1.0f, targetY = 0.0f)

        val cautionSpeedLimit = 0.15f
        val intent = controller.computeMotion(path, pose, goal, speedLimitMps = cautionSpeedLimit)

        assertTrue(
            "Velocity should not exceed clamped speed limit (${intent.linearVelocityMps} <= $cautionSpeedLimit)",
            intent.linearVelocityMps <= cautionSpeedLimit + 0.001f,
        )
    }

    @Test
    fun testGoalArrival_HaltsRoverInsideTolerance() {
        val path = PlannedPath(
            waypoints = listOf(
                Waypoint(0.0f, 0.0f),
                Waypoint(1.0f, 0.0f),
            ),
            totalLengthM = 1.0f,
        )
        // Rover is within 0.10m of goal (tolerance is 0.20m)
        val poseAtGoal = LocalPose(xM = 0.95f, yM = 0.0f, yawDeg = 0.0f)
        val goal = LocalGoal(targetX = 1.0f, targetY = 0.0f, positionToleranceM = 0.20f)

        val intent = controller.computeMotion(path, poseAtGoal, goal)

        assertEquals(MotionDirection.STOP, intent.direction)
        assertEquals(0.0f, intent.linearVelocityMps, 0.001f)
        assertEquals(0.0f, intent.angularVelocityRadS, 0.001f)
        assertEquals("GOAL_REACHED", intent.source)
    }
}
