package com.minesafety.roboeye.vision

import org.junit.Assert.assertEquals
import org.junit.Test

class SceneGeometryAssessorTest {

    private val assessor = SceneGeometryAssessor()

    @Test
    fun `nominal parameters yield TRUSTED geometry`() {
        val assessment = assessor.assess(
            consistency = VisualImuConsistency.CONSISTENT,
            motionState = MotionState.FORWARD_TRANSLATION,
            inlierCount = 18,
            yawRateDps = 2.0f,
            pitchDeg = 2.0f,
            rollDeg = 1.0f,
        )

        assertEquals(GeometryTrustLevel.TRUSTED, assessment.trustLevel)
    }

    @Test
    fun `sensor conflict yields UNTRUSTED geometry`() {
        val assessment = assessor.assess(
            consistency = VisualImuConsistency.CONFLICTING,
            motionState = MotionState.UNKNOWN,
            inlierCount = 18,
            yawRateDps = 2.0f,
            pitchDeg = 0.0f,
            rollDeg = 0.0f,
        )

        assertEquals(GeometryTrustLevel.UNTRUSTED, assessment.trustLevel)
    }

    @Test
    fun `extreme chassis tilt yields DEGRADED geometry`() {
        val assessment = assessor.assess(
            consistency = VisualImuConsistency.CONSISTENT,
            motionState = MotionState.FORWARD_TRANSLATION,
            inlierCount = 18,
            yawRateDps = 2.0f,
            pitchDeg = 26.0f, // > 20 deg
            rollDeg = 0.0f,
        )

        assertEquals(GeometryTrustLevel.DEGRADED, assessment.trustLevel)
    }

    @Test
    fun `rapid yaw turn yields DEGRADED geometry`() {
        val assessment = assessor.assess(
            consistency = VisualImuConsistency.CONSISTENT,
            motionState = MotionState.TURNING_RIGHT,
            inlierCount = 18,
            yawRateDps = 28.0f, // > 16 deg/s
            pitchDeg = 2.0f,
            rollDeg = 0.0f,
        )

        assertEquals(GeometryTrustLevel.DEGRADED, assessment.trustLevel)
    }

    @Test
    fun `insufficient inliers yields UNTRUSTED geometry`() {
        val assessment = assessor.assess(
            consistency = VisualImuConsistency.CONSISTENT,
            motionState = MotionState.FORWARD_TRANSLATION,
            inlierCount = 3, // < 6
            yawRateDps = 0.0f,
            pitchDeg = 0.0f,
            rollDeg = 0.0f,
        )

        assertEquals(GeometryTrustLevel.UNTRUSTED, assessment.trustLevel)
    }
}
