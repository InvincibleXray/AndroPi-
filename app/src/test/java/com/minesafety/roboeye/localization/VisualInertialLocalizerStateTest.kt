package com.minesafety.roboeye.localization

import com.minesafety.roboeye.core.ImuReading
import com.minesafety.roboeye.core.model.CameraFrame
import com.minesafety.roboeye.mapping.Keyframe
import com.minesafety.roboeye.vision.FlowVector
import com.minesafety.roboeye.vision.GeometryTrustLevel
import com.minesafety.roboeye.vision.ImuRotationCompensator
import com.minesafety.roboeye.vision.MotionState
import com.minesafety.roboeye.vision.RansacResult
import com.minesafety.roboeye.vision.RobustFoeEstimator
import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Before
import org.junit.Test
import java.nio.ByteBuffer

class VisualInertialLocalizerStateTest {

    private lateinit var localizer: VisualInertialLocalizer

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
        localizer = VisualInertialLocalizer()
    }

    @Test
    fun stateTransitions_trackingToLost_onVisualFailure() {
        // Step 1: Nominal tracking with inliers
        val inliers = (1..8).map { FlowVector(it * 20f, 100f, it * 20f + 5f, 100f, confidence = 0.9f) }
        val ransacGood = RansacResult(inliers, 0, 5f, 0f, 1.0f, 18)
        val foeGood = makeFoe(320f, 180f, 0.9f, true)
        val compGood = makeComp(inliers, 1.0f, false)

        val pose1 = localizer.update(
            ransacResult = ransacGood,
            foe = foeGood,
            compensatedFlow = compGood,
            motionState = MotionState.FORWARD_TRANSLATION,
            geometryTrust = GeometryTrustLevel.TRUSTED,
            imageWidth = 640,
            imageHeight = 360,
            dtSec = 0.1f,
            timestampNs = 1000L,
            imuReading = null,
            isFreshImu = false,
        )
        assertEquals(TrackingQuality.TRACKING, pose1.trackingState)

        // Step 2: Sudden visual feature failure (0 inliers)
        val ransacBad = RansacResult(emptyList(), 10, 0f, 0f, 0f, 18)
        val foeBad = makeFoe(0f, 0f, 0f, false)
        val compBad = makeComp(emptyList(), 0f, false)

        val pose2 = localizer.update(
            ransacResult = ransacBad,
            foe = foeBad,
            compensatedFlow = compBad,
            motionState = MotionState.VISUAL_TRACKING_LOST,
            geometryTrust = GeometryTrustLevel.UNTRUSTED,
            imageWidth = 640,
            imageHeight = 360,
            dtSec = 0.1f,
            timestampNs = 2000L,
            imuReading = null,
            isFreshImu = false,
        )
        assertEquals(TrackingQuality.LOST, pose2.trackingState)
        assertEquals(pose1.xM, pose2.xM, 0.0001f)
    }

    @Test
    fun stateTransitions_lostToRelocalizingToTracking_onRecovery() {
        val desc = ByteArray(16) { 0xAA.toByte() }
        val kf = Keyframe(
            id = 1,
            pose = LocalPose(xM = 1.0f, yM = 0.0f, yawDeg = 0.0f),
            landmarks = (1..5).map { Landmark(it, 1f, 0f, 100f, 100f, desc) },
            timestampNs = 1000L,
        )

        val dummyRansac = RansacResult(emptyList(), 0, 0f, 0f, 0f, 18)
        val dummyFoe = makeFoe(0f, 0f, 0f, false)
        val dummyComp = makeComp(emptyList(), 0f, false)

        localizer.update(
            ransacResult = dummyRansac,
            foe = dummyFoe,
            compensatedFlow = dummyComp,
            motionState = MotionState.VISUAL_TRACKING_LOST,
            geometryTrust = GeometryTrustLevel.UNTRUSTED,
            imageWidth = 640,
            imageHeight = 360,
            dtSec = 0.1f,
            timestampNs = 1000L,
            imuReading = null,
            isFreshImu = false,
        )
        assertEquals(TrackingQuality.LOST, localizer.localPoseFlow.value.trackingState)

        val relocalResult = localizer.relocalizer.attemptRelocalization(
            currentPose = localizer.localPoseFlow.value,
            visibleLandmarks = kf.landmarks,
            keyframes = listOf(kf),
        )
        assertTrue(relocalResult.isSuccess)
        assertEquals(TrackingQuality.TRACKING, relocalResult.targetPose?.trackingState)
    }

    @Test
    fun staleImu_handlesGracefullyWithDegradedConfidence() {
        val staleImu = ImuReading(
            isAvailable = true,
            yawRateDps = 10.0f,
            timestampNs = 1000L,
        )
        val inliers = (1..6).map { FlowVector(it * 20f, 100f, it * 20f + 2f, 100f, confidence = 0.8f) }
        val ransac = RansacResult(inliers, 0, 2f, 0f, 1.0f, 18)
        val foe = makeFoe(320f, 180f, 0.8f, true)
        val comp = makeComp(inliers, 0.9f, false)

        val pose = localizer.update(
            ransacResult = ransac,
            foe = foe,
            compensatedFlow = comp,
            motionState = MotionState.FORWARD_TRANSLATION,
            geometryTrust = GeometryTrustLevel.TRUSTED,
            imageWidth = 640,
            imageHeight = 360,
            dtSec = 0.1f,
            timestampNs = 1000L,
            imuReading = staleImu,
            isFreshImu = false,
        )

        assertTrue(pose.confidence < 0.90f)
        assertEquals(0.20f, pose.imuConfidence, 0.001f)
    }

    private fun makeBoxFrame(width: Int, height: Int, boxX: Int, boxY: Int, seq: Long): CameraFrame {
        val buf = ByteBuffer.allocate(width * height)
        for (y in 0 until height) {
            for (x in 0 until width) {
                val inBox = (x in boxX until (boxX + 25)) && (y in boxY until (boxY + 25))
                buf.put(if (inBox) 240.toByte() else 20.toByte())
            }
        }
        buf.flip()
        return CameraFrame(
            sequenceNumber = seq,
            timestampNs = seq * 100_000_000L,
            width = width,
            height = height,
            yBuffer = buf,
            yRowStride = width,
            yPixelStride = 1,
        )
    }

    @Test
    fun pipeline_endToEnd_integratesPoseAndMapSummary() {
        val pipeline = com.minesafety.roboeye.vision.GeometricVisionPipeline(
            localizer = localizer,
            spatialMapper = com.minesafety.roboeye.mapping.LocalSpatialMapper(),
        )

        var lastResult: com.minesafety.roboeye.vision.GeometricVisionResult? = null
        pipeline.onGeometricResult = { res ->
            lastResult = res
        }

        val frame1 = makeBoxFrame(120, 120, 40, 40, 1L)
        val frame2 = makeBoxFrame(120, 120, 42, 41, 2L)

        pipeline.processFrame(frame1, null)
        pipeline.processFrame(frame2, null)

        assertTrue(lastResult != null)
        assertTrue(lastResult?.localPose != null)
        assertTrue(lastResult?.mapState != null)
    }
}
