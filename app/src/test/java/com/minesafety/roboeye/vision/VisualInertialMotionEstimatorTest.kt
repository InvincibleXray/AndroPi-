package com.minesafety.roboeye.vision

import com.minesafety.roboeye.core.ImuReading
import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Before
import org.junit.Test

class VisualInertialMotionEstimatorTest {

    private lateinit var estimator: VisualInertialMotionEstimator

    @Before
    fun setUp() {
        estimator = VisualInertialMotionEstimator()
    }

    private fun dummyFlowVector(dx: Float, dy: Float): FlowVector {
        return FlowVector(100f, 100f, 100f + dx, 100f + dy, 0.9f, true)
    }

    @Test
    fun `CASE A - stable forward expansion with low gyro produces FORWARD_TRANSLATION and high confidence`() {
        val inliers = (1..20).map { dummyFlowVector(2.0f, 2.0f) }
        val ransac = RansacResult(inliers, 0, 2.0f, 2.0f, 1.0f, 15)
        val foe = RobustFoeEstimator.FoeResult(x = 320f, y = 180f, isDivergent = true, confidence = 0.95f)
        val compFlow = ImuRotationCompensator.CompensatedFlow(
            compensatedVectors = inliers,
            estimatedRotationalDx = 0f,
            estimatedRotationalDy = 0f,
            yawRateDps = 2.0f,
            pitchRateDps = 1.0f,
            confidenceMultiplier = 1.0f,
            isRotationDominant = false,
        )
        val consistency = VisualImuConsistencyChecker.ConsistencyResult(
            state = VisualImuConsistency.CONSISTENT,
            explanation = "Nominal",
            visualEvidence = 2.0f,
            imuEvidence = 2.0f,
        )
        val imu = ImuReading(gyroZ = 0.03f, axG = 0.05f, isAvailable = true)

        val estimate = estimator.estimate(
            ransacResult = ransac,
            foe = foe,
            compensatedFlow = compFlow,
            consistencyResult = consistency,
            imu = imu,
            isFreshImu = true,
            imageWidth = 640,
            imageHeight = 360,
        )

        assertEquals(MotionState.FORWARD_TRANSLATION, estimate.motionState)
        assertEquals(VisualImuConsistency.CONSISTENT, estimate.consistency)
        assertTrue("Confidence should be high (> 0.7)", estimate.confidence > 0.7f)
    }

    @Test
    fun `CASE B - large rotational flow with large gyro produces TURNING and reduced confidence`() {
        val inliers = (1..15).map { dummyFlowVector(8.0f, 0.0f) }
        val ransac = RansacResult(inliers, 0, 8.0f, 0.0f, 0.9f, 15)
        val foe = RobustFoeEstimator.FoeResult(x = 100f, y = 100f, isDivergent = false, confidence = 0.2f)
        val compFlow = ImuRotationCompensator.CompensatedFlow(
            compensatedVectors = inliers,
            estimatedRotationalDx = 7.5f,
            estimatedRotationalDy = 0f,
            yawRateDps = 28.0f,
            pitchRateDps = 0f,
            confidenceMultiplier = 0.35f,
            isRotationDominant = true,
        )
        val consistency = VisualImuConsistencyChecker.ConsistencyResult(
            state = VisualImuConsistency.CONSISTENT,
            explanation = "Yaw agree",
            visualEvidence = 8.0f,
            imuEvidence = 28.0f,
        )
        val imu = ImuReading(gyroZ = Math.toRadians(28.0).toFloat(), isAvailable = true)

        val estimate = estimator.estimate(
            ransacResult = ransac,
            foe = foe,
            compensatedFlow = compFlow,
            consistencyResult = consistency,
            imu = imu,
            isFreshImu = true,
            imageWidth = 640,
            imageHeight = 360,
        )

        assertEquals(MotionState.TURNING_LEFT, estimate.motionState)
        assertTrue("Confidence should be damped (< 0.45)", estimate.confidence < 0.45f)
    }

    @Test
    fun `CASE C - optical flow reports rotation while IMU reports stationary produces CONFLICTING`() {
        val inliers = (1..15).map { dummyFlowVector(6.0f, 0.0f) }
        val ransac = RansacResult(inliers, 0, 6.0f, 0.0f, 0.9f, 15)
        val foe = RobustFoeEstimator.FoeResult(x = 100f, y = 100f, isDivergent = false, confidence = 0.3f)
        val compFlow = ImuRotationCompensator.CompensatedFlow(
            compensatedVectors = inliers,
            estimatedRotationalDx = 0f,
            estimatedRotationalDy = 0f,
            yawRateDps = 0.5f,
            pitchRateDps = 0f,
            confidenceMultiplier = 1.0f,
            isRotationDominant = false,
        )
        val consistency = VisualImuConsistencyChecker.ConsistencyResult(
            state = VisualImuConsistency.CONFLICTING,
            explanation = "Camera rotation opposed by static gyro",
            visualEvidence = 6.0f,
            imuEvidence = 0.5f,
        )
        val imu = ImuReading(gyroZ = 0.01f, isAvailable = true)

        val estimate = estimator.estimate(
            ransacResult = ransac,
            foe = foe,
            compensatedFlow = compFlow,
            consistencyResult = consistency,
            imu = imu,
            isFreshImu = true,
            imageWidth = 640,
            imageHeight = 360,
        )

        assertEquals(VisualImuConsistency.CONFLICTING, estimate.consistency)
        // Severely penalizes system confidence
        assertTrue("Conflict must lower confidence (< 0.45)", estimate.confidence < 0.45f)
    }

    @Test
    fun `CASE D - camera tracking lost with IMU available produces IMU_ONLY or VISUAL_TRACKING_LOST`() {
        val inliers = emptyList<FlowVector>() // Tracking lost
        val ransac = RansacResult(inliers, 0, 0f, 0f, 0f, 0)
        val foe = RobustFoeEstimator.FoeResult(0f, 0f, false, 0f)
        val compFlow = ImuRotationCompensator.CompensatedFlow(
            compensatedVectors = emptyList(),
            estimatedRotationalDx = 0f,
            estimatedRotationalDy = 0f,
            yawRateDps = 18.0f,
            pitchRateDps = 0f,
            confidenceMultiplier = 1.0f,
            isRotationDominant = false,
        )
        val consistency = VisualImuConsistencyChecker.ConsistencyResult(
            state = VisualImuConsistency.INSUFFICIENT_DATA,
            explanation = "Tracking lost",
            visualEvidence = 0f,
            imuEvidence = 18f,
        )
        val imu = ImuReading(gyroZ = Math.toRadians(18.0).toFloat(), isAvailable = true)

        val estimate = estimator.estimate(
            ransacResult = ransac,
            foe = foe,
            compensatedFlow = compFlow,
            consistencyResult = consistency,
            imu = imu,
            isFreshImu = true,
            imageWidth = 640,
            imageHeight = 360,
        )

        assertEquals(MotionState.IMU_ONLY, estimate.motionState)
        assertTrue(estimate.isImuAvailable)
    }

    @Test
    fun `CASE E - camera and IMU both quiet produces STATIONARY`() {
        val inliers = (1..15).map { dummyFlowVector(0.1f, 0.1f) }
        val ransac = RansacResult(inliers, 0, 0.1f, 0.1f, 1.0f, 5)
        val foe = RobustFoeEstimator.FoeResult(320f, 180f, false, 0.5f)
        val compFlow = ImuRotationCompensator.CompensatedFlow(
            compensatedVectors = inliers,
            estimatedRotationalDx = 0f,
            estimatedRotationalDy = 0f,
            yawRateDps = 1.0f,
            pitchRateDps = 0f,
            confidenceMultiplier = 1.0f,
            isRotationDominant = false,
        )
        val consistency = VisualImuConsistencyChecker.ConsistencyResult(
            state = VisualImuConsistency.CONSISTENT,
            explanation = "Stationary",
            visualEvidence = 0.1f,
            imuEvidence = 1.0f,
        )
        val imu = ImuReading(gyroZ = 0.01f, isAvailable = true)

        val estimate = estimator.estimate(
            ransacResult = ransac,
            foe = foe,
            compensatedFlow = compFlow,
            consistencyResult = consistency,
            imu = imu,
            isFreshImu = true,
            imageWidth = 640,
            imageHeight = 360,
        )

        assertEquals(MotionState.STATIONARY, estimate.motionState)
        assertEquals(VisualImuConsistency.CONSISTENT, estimate.consistency)
    }

    @Test
    fun `CASE F - no valid camera features and no IMU produces UNKNOWN`() {
        val ransac = RansacResult(emptyList(), 0, 0f, 0f, 0f, 0)
        val foe = RobustFoeEstimator.FoeResult(0f, 0f, false, 0f)
        val compFlow = ImuRotationCompensator.CompensatedFlow(
            emptyList(), 0f, 0f, 0f, 0f, 1.0f, false
        )
        val consistency = VisualImuConsistencyChecker.ConsistencyResult(
            VisualImuConsistency.INSUFFICIENT_DATA, "None", 0f, 0f
        )

        val estimate = estimator.estimate(
            ransacResult = ransac,
            foe = foe,
            compensatedFlow = compFlow,
            consistencyResult = consistency,
            imu = null,
            isFreshImu = false,
            imageWidth = 640,
            imageHeight = 360,
        )

        assertEquals(MotionState.UNKNOWN, estimate.motionState)
        assertEquals(0.0f, estimate.confidence, 0.001f)
    }

    @Test
    fun `temporal smoothing ignores single-frame glitch`() {
        val stationaryInliers = (1..12).map { dummyFlowVector(0.1f, 0.1f) }
        val ransacStat = RansacResult(stationaryInliers, 0, 0.1f, 0.1f, 1.0f, 5)
        val foeStat = RobustFoeEstimator.FoeResult(320f, 180f, false, 0.5f)
        val compStat = ImuRotationCompensator.CompensatedFlow(stationaryInliers, 0f, 0f, 1f, 0f, 1f, false)
        val consStat = VisualImuConsistencyChecker.ConsistencyResult(VisualImuConsistency.CONSISTENT, "", 0.1f, 1f)
        val imuStat = ImuReading(gyroZ = 0.01f, isAvailable = true)

        // Frame 1: Stationary
        estimator.estimate(ransacStat, foeStat, compStat, consStat, imuStat, true, 640, 360)
        // Frame 2: Stationary
        val est2 = estimator.estimate(ransacStat, foeStat, compStat, consStat, imuStat, true, 640, 360)
        assertEquals(MotionState.STATIONARY, est2.motionState)

        // Frame 3: Glitch spike to turning right for just 1 frame
        val turningInliers = (1..12).map { dummyFlowVector(-6.0f, 0f) }
        val ransacTurn = RansacResult(turningInliers, 0, -6.0f, 0f, 0.9f, 5)
        val compTurn = ImuRotationCompensator.CompensatedFlow(turningInliers, 0f, 0f, 20f, 0f, 1f, false)
        val consTurn = VisualImuConsistencyChecker.ConsistencyResult(VisualImuConsistency.CONSISTENT, "", 6f, 20f)
        val imuTurn = ImuReading(gyroZ = Math.toRadians(-20.0).toFloat(), isAvailable = true)

        val estGlitch = estimator.estimate(ransacTurn, foeStat, compTurn, consTurn, imuTurn, true, 640, 360)
        // Majority filter over [STATIONARY, STATIONARY, TURNING_RIGHT] retains STATIONARY!
        assertEquals(MotionState.STATIONARY, estGlitch.motionState)
    }

    @Test
    fun `downward optical flow on ground plane produces FORWARD_TRANSLATION even without FOE divergence`() {
        // Floor flow: points move downward (dy = +2.5px) in near-parallel flow, foe.isDivergent = false
        val inliers = (1..20).map { dummyFlowVector(0.0f, 2.5f) }
        val ransac = RansacResult(inliers, 0, 0.0f, 2.5f, 1.0f, 15)
        val foe = RobustFoeEstimator.FoeResult(x = 320f, y = -100f, isDivergent = false, confidence = 0.0f)
        val compFlow = ImuRotationCompensator.CompensatedFlow(
            compensatedVectors = inliers,
            estimatedRotationalDx = 0f,
            estimatedRotationalDy = 0f,
            yawRateDps = 1.0f,
            pitchRateDps = 0f,
            confidenceMultiplier = 1.0f,
            isRotationDominant = false,
        )
        val consistency = VisualImuConsistencyChecker.ConsistencyResult(
            state = VisualImuConsistency.CONSISTENT,
            explanation = "Forward",
            visualEvidence = 2.5f,
            imuEvidence = 1.0f,
        )
        val imu = ImuReading(gyroZ = 0.01f, axG = 0.0f, azG = -0.05f, isAvailable = true)

        val estimate = estimator.estimate(
            ransacResult = ransac,
            foe = foe,
            compensatedFlow = compFlow,
            consistencyResult = consistency,
            imu = imu,
            isFreshImu = true,
            imageWidth = 640,
            imageHeight = 360,
        )

        assertEquals(MotionState.FORWARD_TRANSLATION, estimate.motionState)
        assertTrue(estimate.explanation.contains("downward optical flow"))
    }

    @Test
    fun `upward optical flow on ground plane produces BACKWARD_TRANSLATION`() {
        // Reverse floor flow: points move upward (dy = -2.5px)
        val inliers = (1..20).map { dummyFlowVector(0.0f, -2.5f) }
        val ransac = RansacResult(inliers, 0, 0.0f, -2.5f, 1.0f, 15)
        val foe = RobustFoeEstimator.FoeResult(x = 320f, y = -100f, isDivergent = false, confidence = 0.0f)
        val compFlow = ImuRotationCompensator.CompensatedFlow(
            compensatedVectors = inliers,
            estimatedRotationalDx = 0f,
            estimatedRotationalDy = 0f,
            yawRateDps = 1.0f,
            pitchRateDps = 0f,
            confidenceMultiplier = 1.0f,
            isRotationDominant = false,
        )
        val consistency = VisualImuConsistencyChecker.ConsistencyResult(
            state = VisualImuConsistency.CONSISTENT,
            explanation = "Backward",
            visualEvidence = 2.5f,
            imuEvidence = 1.0f,
        )
        val imu = ImuReading(gyroZ = 0.01f, axG = 0.0f, azG = 0.05f, isAvailable = true)

        val estimate = estimator.estimate(
            ransacResult = ransac,
            foe = foe,
            compensatedFlow = compFlow,
            consistencyResult = consistency,
            imu = imu,
            isFreshImu = true,
            imageWidth = 640,
            imageHeight = 360,
        )

        assertEquals(MotionState.BACKWARD_TRANSLATION, estimate.motionState)
        assertTrue(estimate.explanation.contains("upward optical flow"))
    }

    @Test
    fun `ambiguous flow with non-divergent FOE does not falsely declare BACKWARD_TRANSLATION`() {
        // Lateral or complex flow: dx = 2.0px, dy = 0.1px, non-divergent FOE
        val inliers = (1..20).map { dummyFlowVector(2.0f, 0.1f) }
        val ransac = RansacResult(inliers, 0, 2.0f, 0.1f, 1.0f, 15)
        val foe = RobustFoeEstimator.FoeResult(x = 320f, y = 180f, isDivergent = false, confidence = 0.1f)
        val compFlow = ImuRotationCompensator.CompensatedFlow(
            compensatedVectors = inliers,
            estimatedRotationalDx = 0f,
            estimatedRotationalDy = 0f,
            yawRateDps = 1.0f,
            pitchRateDps = 0f,
            confidenceMultiplier = 1.0f,
            isRotationDominant = false,
        )
        val consistency = VisualImuConsistencyChecker.ConsistencyResult(
            state = VisualImuConsistency.CONSISTENT,
            explanation = "Lateral",
            visualEvidence = 2.0f,
            imuEvidence = 1.0f,
        )
        val imu = ImuReading(gyroZ = 0.01f, isAvailable = true)

        val estimate = estimator.estimate(
            ransacResult = ransac,
            foe = foe,
            compensatedFlow = compFlow,
            consistencyResult = consistency,
            imu = imu,
            isFreshImu = true,
            imageWidth = 640,
            imageHeight = 360,
        )

        // Must NOT be BACKWARD_TRANSLATION!
        org.junit.Assert.assertNotEquals(MotionState.BACKWARD_TRANSLATION, estimate.motionState)
        assertEquals(MotionState.COMBINED_MOTION, estimate.motionState)
    }

    @Test
    fun `continuous forward motion maintains FORWARD_TRANSLATION through single-frame stationary dropouts`() {
        val forwardInliers = (1..20).map { dummyFlowVector(0.0f, 2.5f) }
        val ransacFwd = RansacResult(forwardInliers, 0, 0.0f, 2.5f, 1.0f, 15)
        val foeFwd = RobustFoeEstimator.FoeResult(320f, -100f, false, 0f)
        val compFwd = ImuRotationCompensator.CompensatedFlow(forwardInliers, 0f, 0f, 1f, 0f, 1f, false)
        val consFwd = VisualImuConsistencyChecker.ConsistencyResult(VisualImuConsistency.CONSISTENT, "", 2.5f, 1f)
        val imuFwd = ImuReading(gyroZ = 0.01f, axG = 0f, azG = -0.05f, isAvailable = true)

        val statInliers = (1..15).map { dummyFlowVector(0.1f, 0.1f) }
        val ransacStat = RansacResult(statInliers, 0, 0.1f, 0.1f, 1.0f, 5)
        val foeStat = RobustFoeEstimator.FoeResult(320f, 180f, false, 0.5f)
        val compStat = ImuRotationCompensator.CompensatedFlow(statInliers, 0f, 0f, 1f, 0f, 1f, false)
        val consStat = VisualImuConsistencyChecker.ConsistencyResult(VisualImuConsistency.CONSISTENT, "", 0.1f, 1f)
        val imuStat = ImuReading(gyroZ = 0.01f, isAvailable = true)

        // Frame 1 & 2: Forward Translation
        val e1 = estimator.estimate(ransacFwd, foeFwd, compFwd, consFwd, imuFwd, true, 640, 360)
        assertEquals(MotionState.FORWARD_TRANSLATION, e1.motionState)
        val e2 = estimator.estimate(ransacFwd, foeFwd, compFwd, consFwd, imuFwd, true, 640, 360)
        assertEquals(MotionState.FORWARD_TRANSLATION, e2.motionState)

        // Frame 3: Single-frame stationary dropout (e.g. low texture floor)
        val e3 = estimator.estimate(ransacStat, foeStat, compStat, consStat, imuStat, true, 640, 360)
        assertEquals("Hysteresis must hold FORWARD_TRANSLATION across 1-frame stationary glitch", MotionState.FORWARD_TRANSLATION, e3.motionState)

        // Frame 4: Forward resumes
        val e4 = estimator.estimate(ransacFwd, foeFwd, compFwd, consFwd, imuFwd, true, 640, 360)
        assertEquals(MotionState.FORWARD_TRANSLATION, e4.motionState)

        // Frame 5 & 6: Two-frame stationary dropout
        val e5 = estimator.estimate(ransacStat, foeStat, compStat, consStat, imuStat, true, 640, 360)
        assertEquals("Hysteresis must hold FORWARD_TRANSLATION across 2-frame stationary dropout", MotionState.FORWARD_TRANSLATION, e5.motionState)
        val e6 = estimator.estimate(ransacStat, foeStat, compStat, consStat, imuStat, true, 640, 360)
        assertEquals("Hysteresis must still hold FORWARD_TRANSLATION at 2nd quiet frame", MotionState.FORWARD_TRANSLATION, e6.motionState)

        // Frame 7: Forward resumes
        val e7 = estimator.estimate(ransacFwd, foeFwd, compFwd, consFwd, imuFwd, true, 640, 360)
        assertEquals(MotionState.FORWARD_TRANSLATION, e7.motionState)
    }

    @Test
    fun `sustained stopping transitions from FORWARD_TRANSLATION to STATIONARY after 3 consecutive quiet frames`() {
        val forwardInliers = (1..20).map { dummyFlowVector(0.0f, 2.5f) }
        val ransacFwd = RansacResult(forwardInliers, 0, 0.0f, 2.5f, 1.0f, 15)
        val foeFwd = RobustFoeEstimator.FoeResult(320f, -100f, false, 0f)
        val compFwd = ImuRotationCompensator.CompensatedFlow(forwardInliers, 0f, 0f, 1f, 0f, 1f, false)
        val consFwd = VisualImuConsistencyChecker.ConsistencyResult(VisualImuConsistency.CONSISTENT, "", 2.5f, 1f)
        val imuFwd = ImuReading(gyroZ = 0.01f, axG = 0f, azG = -0.05f, isAvailable = true)

        val statInliers = (1..15).map { dummyFlowVector(0.1f, 0.1f) }
        val ransacStat = RansacResult(statInliers, 0, 0.1f, 0.1f, 1.0f, 5)
        val foeStat = RobustFoeEstimator.FoeResult(320f, 180f, false, 0.5f)
        val compStat = ImuRotationCompensator.CompensatedFlow(statInliers, 0f, 0f, 1f, 0f, 1f, false)
        val consStat = VisualImuConsistencyChecker.ConsistencyResult(VisualImuConsistency.CONSISTENT, "", 0.1f, 1f)
        val imuStat = ImuReading(gyroZ = 0.01f, isAvailable = true)

        // Establish Forward Translation
        estimator.estimate(ransacFwd, foeFwd, compFwd, consFwd, imuFwd, true, 640, 360)
        estimator.estimate(ransacFwd, foeFwd, compFwd, consFwd, imuFwd, true, 640, 360)

        // Frame 1 quiet: held
        val s1 = estimator.estimate(ransacStat, foeStat, compStat, consStat, imuStat, true, 640, 360)
        assertEquals(MotionState.FORWARD_TRANSLATION, s1.motionState)

        // Frame 2 quiet: held
        val s2 = estimator.estimate(ransacStat, foeStat, compStat, consStat, imuStat, true, 640, 360)
        assertEquals(MotionState.FORWARD_TRANSLATION, s2.motionState)

        // Frame 3 quiet: confirmed STOP
        val s3 = estimator.estimate(ransacStat, foeStat, compStat, consStat, imuStat, true, 640, 360)
        assertEquals(MotionState.STATIONARY, s3.motionState)
    }

    @Test
    fun `direction reversal from Forward to Backward transitions at exactly 3 consecutive opposing frames`() {
        val forwardInliers = (1..20).map { dummyFlowVector(0.0f, 2.5f) }
        val ransacFwd = RansacResult(forwardInliers, 0, 0.0f, 2.5f, 1.0f, 15)
        val foeFwd = RobustFoeEstimator.FoeResult(320f, -100f, false, 0f)
        val compFwd = ImuRotationCompensator.CompensatedFlow(forwardInliers, 0f, 0f, 1f, 0f, 1f, false)
        val consFwd = VisualImuConsistencyChecker.ConsistencyResult(VisualImuConsistency.CONSISTENT, "", 2.5f, 1f)
        val imuFwd = ImuReading(gyroZ = 0.01f, axG = 0f, azG = -0.05f, isAvailable = true)

        val bwdInliers = (1..20).map { dummyFlowVector(0.0f, -2.5f) }
        val ransacBwd = RansacResult(bwdInliers, 0, 0.0f, -2.5f, 1.0f, 15)
        val foeBwd = RobustFoeEstimator.FoeResult(320f, -100f, false, 0f)
        val compBwd = ImuRotationCompensator.CompensatedFlow(bwdInliers, 0f, 0f, 1f, 0f, 1f, false)
        val consBwd = VisualImuConsistencyChecker.ConsistencyResult(VisualImuConsistency.CONSISTENT, "", 2.5f, 1f)
        val imuBwd = ImuReading(gyroZ = 0.01f, axG = 0f, azG = 0.05f, isAvailable = true)

        // Establish Forward
        estimator.estimate(ransacFwd, foeFwd, compFwd, consFwd, imuFwd, true, 640, 360)
        estimator.estimate(ransacFwd, foeFwd, compFwd, consFwd, imuFwd, true, 640, 360)

        // Frame 1 opposing backward: held to avoid single-frame bounce
        val r1 = estimator.estimate(ransacBwd, foeBwd, compBwd, consBwd, imuBwd, true, 640, 360)
        assertEquals(MotionState.FORWARD_TRANSLATION, r1.motionState)

        // Frame 2 opposing backward: held
        val r2 = estimator.estimate(ransacBwd, foeBwd, compBwd, consBwd, imuBwd, true, 640, 360)
        assertEquals(MotionState.FORWARD_TRANSLATION, r2.motionState)

        // Frame 3 opposing backward: decisive transition to BACKWARD_TRANSLATION
        val r3 = estimator.estimate(ransacBwd, foeBwd, compBwd, consBwd, imuBwd, true, 640, 360)
        assertEquals(MotionState.BACKWARD_TRANSLATION, r3.motionState)
    }

    @Test
    fun `transient tracking glitch does not abort confirmed FORWARD_TRANSLATION`() {
        val forwardInliers = (1..20).map { dummyFlowVector(0.0f, 2.5f) }
        val ransacFwd = RansacResult(forwardInliers, 0, 0.0f, 2.5f, 1.0f, 15)
        val foeFwd = RobustFoeEstimator.FoeResult(320f, -100f, false, 0f)
        val compFwd = ImuRotationCompensator.CompensatedFlow(forwardInliers, 0f, 0f, 1f, 0f, 1f, false)
        val consFwd = VisualImuConsistencyChecker.ConsistencyResult(VisualImuConsistency.CONSISTENT, "", 2.5f, 1f)
        val imuFwd = ImuReading(gyroZ = 0.01f, axG = 0f, azG = -0.05f, isAvailable = true)

        // Lost tracking glitch
        val ransacLost = RansacResult(emptyList(), 0, 0f, 0f, 0f, 0)
        val foeLost = RobustFoeEstimator.FoeResult(0f, 0f, false, 0f)
        val compLost = ImuRotationCompensator.CompensatedFlow(emptyList(), 0f, 0f, 0f, 0f, 1f, false)
        val consLost = VisualImuConsistencyChecker.ConsistencyResult(VisualImuConsistency.INSUFFICIENT_DATA, "", 0f, 0f)

        // Establish Forward
        estimator.estimate(ransacFwd, foeFwd, compFwd, consFwd, imuFwd, true, 640, 360)
        estimator.estimate(ransacFwd, foeFwd, compFwd, consFwd, imuFwd, true, 640, 360)

        // Single glitch frame: should hold active forward motion
        val g1 = estimator.estimate(ransacLost, foeLost, compLost, consLost, imuFwd, true, 640, 360)
        assertEquals(MotionState.FORWARD_TRANSLATION, g1.motionState)

        // Forward resumes
        val g2 = estimator.estimate(ransacFwd, foeFwd, compFwd, consFwd, imuFwd, true, 640, 360)
        assertEquals(MotionState.FORWARD_TRANSLATION, g2.motionState)
    }
}
