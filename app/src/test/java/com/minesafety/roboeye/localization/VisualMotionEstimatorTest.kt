package com.minesafety.roboeye.localization

import com.minesafety.roboeye.vision.FlowVector
import com.minesafety.roboeye.vision.GeometryTrustLevel
import com.minesafety.roboeye.vision.ImuRotationCompensator
import com.minesafety.roboeye.vision.MotionState
import com.minesafety.roboeye.vision.RansacResult
import com.minesafety.roboeye.vision.RobustFoeEstimator
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Before
import org.junit.Test

class VisualMotionEstimatorTest {

    private lateinit var estimator: VisualMotionEstimator

    private fun makeFoe(x: Float, y: Float, conf: Float, div: Boolean): RobustFoeEstimator.FoeResult {
        return RobustFoeEstimator.FoeResult(x = x, y = y, isDivergent = div, confidence = conf)
    }

    private fun makeComp(vectors: List<FlowVector>, confMult: Float, rotDom: Boolean): ImuRotationCompensator.CompensatedFlow {
        return ImuRotationCompensator.CompensatedFlow(
            compensatedVectors = vectors,
            estimatedRotationalDx = 0f,
            estimatedRotationalDy = 0f,
            yawRateDps = 0f,
            pitchRateDps = 0f,
            confidenceMultiplier = confMult,
            isRotationDominant = rotDom,
        )
    }

    @Before
    fun setUp() {
        estimator = VisualMotionEstimator(
            focalLengthScale = 0.85f,
            nominalGroundDistanceM = 2.0f,
            minInliersForMotion = 3,
        )
    }

    @Test
    fun stationaryMotion_producesZeroDisplacement_andHighConfidence() {
        val ransac = RansacResult(
            inliers = listOf(
                FlowVector(100f, 100f, 100.1f, 100f, confidence = 0.9f),
                FlowVector(200f, 100f, 200f, 100.1f, confidence = 0.9f),
                FlowVector(150f, 150f, 150.1f, 150.1f, confidence = 0.9f),
            ),
            outliersCount = 0,
            consensusDx = 0.05f,
            consensusDy = 0.05f,
            inlierRatio = 1.0f,
            iterationsRun = 18,
        )
        val foe = makeFoe(160f, 120f, conf = 0.8f, div = false)
        val comp = makeComp(ransac.inliers, 1.0f, false)

        val motion = estimator.estimate(
            ransacResult = ransac,
            foe = foe,
            compensatedFlow = comp,
            motionState = MotionState.STATIONARY,
            geometryTrust = GeometryTrustLevel.TRUSTED,
            imageWidth = 640,
            imageHeight = 360,
            dtSec = 0.1f,
            timestampNs = 1000L,
        )

        assertTrue(motion.isValid)
        assertEquals(0.0f, motion.deltaXM, 0.001f)
        assertEquals(0.0f, motion.deltaYM, 0.001f)
        assertTrue(motion.translationConfidence >= 0.85f)
    }

    @Test
    fun forwardExpansion_producesForwardTranslation_andScaleEstimated() {
        val foeX = 320f
        val foeY = 180f
        val inliers = listOf(
            FlowVector(prevX = 200f, prevY = 180f, currX = 180f, currY = 180f, confidence = 0.9f),
            FlowVector(prevX = 440f, prevY = 180f, currX = 460f, currY = 180f, confidence = 0.9f),
            FlowVector(prevX = 320f, prevY = 80f, currX = 320f, currY = 60f, confidence = 0.9f),
            FlowVector(prevX = 320f, prevY = 280f, currX = 320f, currY = 300f, confidence = 0.9f),
            FlowVector(prevX = 240f, prevY = 120f, currX = 220f, currY = 100f, confidence = 0.9f),
            FlowVector(prevX = 400f, prevY = 240f, currX = 420f, currY = 260f, confidence = 0.9f),
            FlowVector(prevX = 240f, prevY = 240f, currX = 220f, currY = 260f, confidence = 0.9f),
            FlowVector(prevX = 400f, prevY = 120f, currX = 420f, currY = 100f, confidence = 0.9f),
        )
        val ransac = RansacResult(inliers, 0, 0f, 0f, 1.0f, 18)
        val foe = makeFoe(foeX, foeY, conf = 0.9f, div = true)
        val comp = makeComp(inliers, 1.0f, false)

        val motion = estimator.estimate(
            ransacResult = ransac,
            foe = foe,
            compensatedFlow = comp,
            motionState = MotionState.FORWARD_TRANSLATION,
            geometryTrust = GeometryTrustLevel.TRUSTED,
            imageWidth = 640,
            imageHeight = 360,
            dtSec = 0.1f,
            timestampNs = 2000L,
        )

        assertTrue(motion.isValid)
        assertTrue(motion.deltaXM > 0.0f)
        assertEquals(MonocularScaleState.SCALE_ESTIMATED, motion.scaleState)
    }

    @Test
    fun pureRotation_producesVisualYaw_andLowTranslation() {
        val inliers = listOf(
            FlowVector(100f, 100f, 130f, 100f, confidence = 0.9f),
            FlowVector(200f, 100f, 230f, 100f, confidence = 0.9f),
            FlowVector(300f, 100f, 330f, 100f, confidence = 0.9f),
            FlowVector(400f, 100f, 430f, 100f, confidence = 0.9f),
        )
        val ransac = RansacResult(inliers, 0, consensusDx = 30f, consensusDy = 0f, inlierRatio = 1.0f, iterationsRun = 18)
        val foe = makeFoe(320f, 180f, conf = 0.2f, div = false)
        val comp = makeComp(inliers, 0.4f, true)

        val motion = estimator.estimate(
            ransacResult = ransac,
            foe = foe,
            compensatedFlow = comp,
            motionState = MotionState.TURNING_LEFT,
            geometryTrust = GeometryTrustLevel.DEGRADED,
            imageWidth = 640,
            imageHeight = 360,
            dtSec = 0.1f,
            timestampNs = 3000L,
        )

        assertTrue(motion.isValid)
        assertTrue(motion.deltaYawRad < 0.0f)
        assertTrue(motion.deltaXM <= 0.01f)
    }

    @Test
    fun combinedMotion_producesForwardAndTurn() {
        val inliers = (1..8).map {
            FlowVector(100f * it, 100f, 100f * it + 15f, 110f, confidence = 0.85f)
        }
        val ransac = RansacResult(inliers, 0, consensusDx = 15f, consensusDy = 10f, inlierRatio = 0.9f, iterationsRun = 18)
        val foe = makeFoe(300f, 160f, conf = 0.7f, div = true)
        val comp = makeComp(inliers, 0.8f, false)

        val motion = estimator.estimate(
            ransacResult = ransac,
            foe = foe,
            compensatedFlow = comp,
            motionState = MotionState.COMBINED_MOTION,
            geometryTrust = GeometryTrustLevel.TRUSTED,
            imageWidth = 640,
            imageHeight = 360,
            dtSec = 0.1f,
            timestampNs = 4000L,
        )

        assertTrue(motion.isValid)
        assertTrue(motion.deltaXM >= 0f)
        assertTrue(motion.deltaYawRad != 0f)
    }

    @Test
    fun insufficientInliers_producesInvalidMotion() {
        val ransac = RansacResult(
            inliers = listOf(
                FlowVector(100f, 100f, 105f, 100f, confidence = 0.5f)
            ),
            outliersCount = 10,
            consensusDx = 5f,
            consensusDy = 0f,
            inlierRatio = 0.09f,
            iterationsRun = 18,
        )
        val foe = makeFoe(0f, 0f, conf = 0f, div = false)
        val comp = makeComp(ransac.inliers, 0.1f, false)

        val motion = estimator.estimate(
            ransacResult = ransac,
            foe = foe,
            compensatedFlow = comp,
            motionState = MotionState.VISUAL_TRACKING_LOST,
            geometryTrust = GeometryTrustLevel.UNTRUSTED,
            imageWidth = 640,
            imageHeight = 360,
            dtSec = 0.1f,
            timestampNs = 5000L,
        )

        assertFalse(motion.isValid)
        assertEquals(0.0f, motion.overallConfidence, 0.001f)
        assertEquals(MonocularScaleState.SCALE_UNKNOWN, motion.scaleState)
    }

    @Test
    fun monocularScale_trustedGeometryProducesScaleEstimated_untrustedProducesScaleUnknown() {
        val inliers = (1..10).map {
            FlowVector(50f * it, 50f * it, 50f * it + 5f, 50f * it + 5f, confidence = 0.9f)
        }
        val ransac = RansacResult(inliers, 0, 5f, 5f, 1.0f, 18)
        val foe = makeFoe(320f, 180f, conf = 0.8f, div = true)
        val comp = makeComp(inliers, 1.0f, false)

        val trustedMotion = estimator.estimate(
            ransacResult = ransac,
            foe = foe,
            compensatedFlow = comp,
            motionState = MotionState.FORWARD_TRANSLATION,
            geometryTrust = GeometryTrustLevel.TRUSTED,
            imageWidth = 640,
            imageHeight = 360,
            dtSec = 0.1f,
            timestampNs = 6000L,
        )
        assertEquals(MonocularScaleState.SCALE_ESTIMATED, trustedMotion.scaleState)

        val untrustedMotion = estimator.estimate(
            ransacResult = ransac,
            foe = foe,
            compensatedFlow = comp,
            motionState = MotionState.FORWARD_TRANSLATION,
            geometryTrust = GeometryTrustLevel.UNTRUSTED,
            imageWidth = 640,
            imageHeight = 360,
            dtSec = 0.1f,
            timestampNs = 6000L,
        )
        assertEquals(MonocularScaleState.SCALE_UNKNOWN, untrustedMotion.scaleState)
    }

    @Test
    fun downwardGroundFlow_withoutFoeDivergence_producesPositiveForwardTranslation() {
        val inliers = (1..10).map {
            FlowVector(100f, 50f * it, 100f, 50f * it + 3.0f, confidence = 0.9f)
        }
        val ransac = RansacResult(inliers, 0, 0.0f, 3.0f, 1.0f, 18)
        // Parallel downward flow: FOE is non-divergent or off-screen
        val foe = makeFoe(320f, -100f, conf = 0.0f, div = false)
        val comp = makeComp(inliers, 1.0f, false)

        val motion = estimator.estimate(
            ransacResult = ransac,
            foe = foe,
            compensatedFlow = comp,
            motionState = MotionState.FORWARD_TRANSLATION,
            geometryTrust = GeometryTrustLevel.TRUSTED,
            imageWidth = 640,
            imageHeight = 360,
            dtSec = 0.1f,
            timestampNs = 7000L,
        )

        assertTrue(motion.isValid)
        assertTrue("deltaXM must be positive for forward downward flow", motion.deltaXM > 0.0f)
        assertTrue("translationConfidence must be positive", motion.translationConfidence >= 0.5f)
    }

    @Test
    fun upwardGroundFlow_producesNegativeBackwardTranslation() {
        val inliers = (1..10).map {
            FlowVector(100f, 50f * it, 100f, 50f * it - 3.0f, confidence = 0.9f)
        }
        val ransac = RansacResult(inliers, 0, 0.0f, -3.0f, 1.0f, 18)
        val foe = makeFoe(320f, -100f, conf = 0.0f, div = false)
        val comp = makeComp(inliers, 1.0f, false)

        val motion = estimator.estimate(
            ransacResult = ransac,
            foe = foe,
            compensatedFlow = comp,
            motionState = MotionState.BACKWARD_TRANSLATION,
            geometryTrust = GeometryTrustLevel.TRUSTED,
            imageWidth = 640,
            imageHeight = 360,
            dtSec = 0.1f,
            timestampNs = 8000L,
        )

        assertTrue(motion.isValid)
        assertTrue("deltaXM must be negative for backward motion", motion.deltaXM < 0.0f)
    }
}
