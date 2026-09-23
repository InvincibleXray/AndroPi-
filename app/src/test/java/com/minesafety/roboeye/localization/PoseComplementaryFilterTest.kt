package com.minesafety.roboeye.localization

import com.minesafety.roboeye.core.ImuReading
import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Before
import org.junit.Test
import kotlin.math.abs

class PoseComplementaryFilterTest {

    private lateinit var filter: PoseComplementaryFilter

    @Before
    fun setUp() {
        filter = PoseComplementaryFilter(
            alphaVisualYaw = 0.08f,
            maxStepTranslationM = 0.35f,
            maxStepRotationDeg = 30.0f,
            zuptAccelThresholdMps2 = 0.15f,
            zuptGyroThresholdDps = 2.0f,
        )
    }

    @Test
    fun originInitialization_startsAtZeroWithFullConfidence() {
        val origin = LocalPose.ORIGIN
        assertEquals(0.0f, origin.xM, 0.001f)
        assertEquals(0.0f, origin.yM, 0.001f)
        assertEquals(0.0f, origin.yawDeg, 0.001f)
        assertEquals(1.0f, origin.confidence, 0.001f)
        assertEquals(TrackingQuality.INITIALIZING, origin.trackingState)
    }

    @Test
    fun pureGyroYawIntegration_whenVisionInvalid() {
        val imu = ImuReading(
            isAvailable = true,
            yawRateDps = 20.0f, // 20 deg/sec
            timestampNs = 1_000_000_000L,
        )
        val visualMotion = VisualRelativeMotion.ZERO // invalid vision

        val pose = filter.update(
            prevPose = LocalPose.ORIGIN,
            visualMotion = visualMotion,
            imu = imu,
            isFreshImu = true,
            dtSec = 0.1f, // expected turn = 2.0 deg
            timestampNs = 1_000_000_000L,
        )

        assertEquals(2.0f, pose.yawDeg, 0.1f)
        assertEquals(0.0f, pose.xM, 0.001f)
        assertEquals(0.0f, pose.yM, 0.001f)
        assertEquals(TrackingQuality.DEGRADED, pose.trackingState)
    }

    @Test
    fun pureVisualYawIntegration_whenImuUnavailable() {
        val visualMotion = VisualRelativeMotion(
            timestampNs = 1000L,
            deltaXM = 0.10f,
            deltaYM = 0.0f,
            deltaYawRad = Math.toRadians(5.0).toFloat(), // 5 degrees CCW
            translationConfidence = 0.8f,
            rotationConfidence = 0.8f,
            overallConfidence = 0.8f,
            scaleState = MonocularScaleState.SCALE_ESTIMATED,
            isValid = true,
        )

        val pose = filter.update(
            prevPose = LocalPose.ORIGIN,
            visualMotion = visualMotion,
            imu = null,
            isFreshImu = false,
            dtSec = 0.1f,
            timestampNs = 1000L,
        )

        assertEquals(5.0f, pose.yawDeg, 0.1f)
        assertTrue(pose.xM > 0.0f)
        assertTrue(pose.confidence < 0.90f) // Degraded due to missing IMU
    }

    @Test
    fun fusedYaw_combinesGyroAndVisualYaw() {
        // Gyro reports 10.0 deg (dt=0.1s, rate=100 dps)
        val imu = ImuReading(
            isAvailable = true,
            yawRateDps = 100.0f,
            timestampNs = 1000L,
        )
        // Vision reports 5.0 deg
        val visualMotion = VisualRelativeMotion(
            timestampNs = 1000L,
            deltaXM = 0.05f,
            deltaYM = 0.0f,
            deltaYawRad = Math.toRadians(5.0).toFloat(),
            translationConfidence = 0.9f,
            rotationConfidence = 0.9f,
            overallConfidence = 0.9f,
            scaleState = MonocularScaleState.SCALE_ESTIMATED,
            isValid = true,
        )

        val pose = filter.update(
            prevPose = LocalPose.ORIGIN,
            visualMotion = visualMotion,
            imu = imu,
            isFreshImu = true,
            dtSec = 0.1f,
            timestampNs = 1000L,
        )

        // Expected = (1 - 0.08) * 10.0 + 0.08 * 5.0 = 9.2 + 0.4 = 9.6 deg
        assertEquals(9.6f, pose.yawDeg, 0.2f)
        assertEquals(TrackingQuality.TRACKING, pose.trackingState)
    }

    @Test
    fun zupt_stationaryDetection_haltsDrift() {
        val quietImu = ImuReading(
            isAvailable = true,
            yawRateDps = 0.2f, // below 2.0 dps
            axG = 0.005f, // below 0.15 m/s2
            ayG = 0.005f,
            timestampNs = 1000L,
        )
        // Visual motion with small noisy drift
        val visualMotion = VisualRelativeMotion(
            timestampNs = 1000L,
            deltaXM = 0.02f,
            deltaYM = 0.01f,
            deltaYawRad = 0.001f,
            translationConfidence = 0.4f,
            rotationConfidence = 0.4f,
            overallConfidence = 0.4f,
            scaleState = MonocularScaleState.SCALE_UNKNOWN,
            isValid = true,
        )

        val pose = filter.update(
            prevPose = LocalPose.ORIGIN,
            visualMotion = visualMotion,
            imu = quietImu,
            isFreshImu = true,
            dtSec = 0.1f,
            timestampNs = 1000L,
        )

        // ZUPT clamps translation to strictly zero
        assertEquals(0.0f, pose.xM, 0.0001f)
        assertEquals(0.0f, pose.yM, 0.0001f)
    }

    @Test
    fun poseJumpProtection_clampsLinearDisplacement() {
        val wildJump = VisualRelativeMotion(
            timestampNs = 1000L,
            deltaXM = 2.50f, // impossible 2.5m in 0.1s!
            deltaYM = 0.0f,
            deltaYawRad = 0.0f,
            translationConfidence = 0.8f,
            rotationConfidence = 0.8f,
            overallConfidence = 0.8f,
            scaleState = MonocularScaleState.SCALE_ESTIMATED,
            isValid = true,
        )

        val pose = filter.update(
            prevPose = LocalPose.ORIGIN,
            visualMotion = wildJump,
            imu = null,
            isFreshImu = false,
            dtSec = 0.1f,
            timestampNs = 1000L,
        )

        // Clamped to maxStepTranslationM = 0.35m
        assertEquals(0.35f, pose.xM, 0.01f)
        assertEquals(TrackingQuality.DEGRADED, pose.trackingState)
        assertTrue(pose.confidence < 0.60f)
    }

    @Test
    fun poseJumpProtection_clampsAngularDisplacement() {
        val imu = ImuReading(
            isAvailable = true,
            yawRateDps = 720.0f, // 720 dps = 72 deg in 0.1s
            timestampNs = 1000L,
        )
        val motion = VisualRelativeMotion.ZERO

        val pose = filter.update(
            prevPose = LocalPose.ORIGIN,
            visualMotion = motion,
            imu = imu,
            isFreshImu = true,
            dtSec = 0.1f,
            timestampNs = 1000L,
        )

        // Clamped to maxStepRotationDeg = 30.0 deg
        assertEquals(30.0f, abs(pose.yawDeg), 0.1f)
    }
}
