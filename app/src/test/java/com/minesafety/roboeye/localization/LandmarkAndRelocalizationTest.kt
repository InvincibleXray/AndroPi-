package com.minesafety.roboeye.localization

import com.minesafety.roboeye.core.model.CameraFrame
import com.minesafety.roboeye.mapping.Keyframe
import com.minesafety.roboeye.vision.FlowVector
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Before
import org.junit.Test
import java.nio.ByteBuffer

class LandmarkAndRelocalizationTest {

    private lateinit var landmarkManager: LandmarkManager
    private lateinit var relocalizer: LocalRelocalizer

    @Before
    fun setUp() {
        landmarkManager = LandmarkManager(maxCapacity = 10)
        relocalizer = LocalRelocalizer(maxHammingDistance = 42, minMatchingLandmarks = 3)
    }

    @Test
    fun landmarkCreation_extracts16ByteDescriptor() {
        val yBuf = ByteArray(640 * 360) { (it % 255).toByte() }
        val frame = CameraFrame(
            yBuffer = ByteBuffer.wrap(yBuf),
            width = 640,
            height = 360,
            timestampNs = 1000L,
            sequenceNumber = 1L,
        )

        val desc = landmarkManager.extractDescriptor(frame, 320, 180)
        assertEquals(16, desc.size)
    }

    @Test
    fun landmarkTracking_updatesPositionAndCount() {
        val yBuf = ByteArray(640 * 360) { 100.toByte() }
        val frame = CameraFrame(
            yBuffer = ByteBuffer.wrap(yBuf),
            width = 640,
            height = 360,
            timestampNs = 1000L,
            sequenceNumber = 1L,
        )

        val inliersPass1 = listOf(
            FlowVector(100f, 100f, 101f, 100f, confidence = 0.9f)
        )
        val lms1 = landmarkManager.update(inliersPass1, LocalPose.ORIGIN, frame, 1000L)
        assertEquals(1, lms1.size)
        assertEquals(1, lms1[0].observationCount)

        // Pass 2: point observed nearby
        val inliersPass2 = listOf(
            FlowVector(101f, 100f, 103f, 100f, confidence = 0.9f)
        )
        val lms2 = landmarkManager.update(inliersPass2, LocalPose.ORIGIN, frame, 2000L)
        assertEquals(1, lms2.size)
        assertEquals(2, lms2[0].observationCount)
        assertEquals(103f, lms2[0].imageX, 0.1f)
    }

    @Test
    fun landmarkPruning_enforcesHardCapacityBound() {
        val yBuf = ByteArray(640 * 360) { 120.toByte() }
        val frame = CameraFrame(
            yBuffer = ByteBuffer.wrap(yBuf),
            width = 640,
            height = 360,
            timestampNs = 1000L,
            sequenceNumber = 1L,
        )

        // Add 15 distinct features to capacity-10 manager
        val inliers = (1..15).map {
            FlowVector(it * 30f, 100f, it * 30f, 100f, confidence = 0.9f)
        }
        val lms = landmarkManager.update(inliers, LocalPose.ORIGIN, frame, 1000L)

        // Capacity bound = 10
        assertEquals(10, lms.size)
    }

    @Test
    fun keyframeCreation_triggeredByTranslationOrRotation() {
        val map = com.minesafety.roboeye.mapping.LocalSpatialMap(maxKeyframes = 5)
        val p0 = LocalPose.ORIGIN
        val lms = listOf(
            Landmark(1, 1f, 1f, 100f, 100f, ByteArray(16))
        )

        // Initial keyframe created
        val created0 = map.maybeCreateKeyframe(p0, lms, 1000L)
        assertTrue(created0)
        assertEquals(1, map.storedKeyframes.size)

        // Small movement (0.05m, 1 deg) does not trigger keyframe
        val pSmall = p0.copy(xM = 0.05f, yawDeg = 1.0f)
        val created1 = map.maybeCreateKeyframe(pSmall, lms, 2000L)
        assertFalse(created1)

        // Large movement (0.50m) triggers keyframe
        val pBig = p0.copy(xM = 0.50f)
        val created2 = map.maybeCreateKeyframe(pBig, lms, 3000L)
        assertTrue(created2)
        assertEquals(2, map.storedKeyframes.size)
    }

    @Test
    fun keyframePruning_enforcesHardCapacityBound() {
        val map = com.minesafety.roboeye.mapping.LocalSpatialMap(maxKeyframes = 3)
        val lms = emptyList<Landmark>()

        for (i in 1..5) {
            map.maybeCreateKeyframe(LocalPose.ORIGIN.copy(xM = i * 1.0f), lms, i * 1000L)
        }

        // Bounded to 3 keyframes
        assertEquals(3, map.storedKeyframes.size)
    }

    @Test
    fun relocalization_succeedsOnMatchingLandmarks() {
        val sharedDesc = ByteArray(16) { 0x55.toByte() }
        val kfLandmarks = (1..5).map {
            Landmark(it, it.toFloat(), 0f, 100f, 100f, sharedDesc.clone())
        }
        val keyframe = Keyframe(
            id = 1,
            pose = LocalPose(xM = 2.0f, yM = 0.5f, yawDeg = 15.0f),
            landmarks = kfLandmarks,
            timestampNs = 1000L,
        )

        // Current rover is LOST with similar landmarks
        val currentLandmarks = (1..4).map {
            Landmark(it + 10, 0f, 0f, 105f, 105f, sharedDesc.clone())
        }
        val lostPose = LocalPose(
            xM = 1.8f,
            yM = 0.4f,
            yawDeg = 12.0f,
            trackingState = TrackingQuality.LOST,
        )

        val result = relocalizer.attemptRelocalization(
            currentPose = lostPose,
            visibleLandmarks = currentLandmarks,
            keyframes = listOf(keyframe),
        )

        assertTrue(result.isSuccess)
        assertEquals(1, result.matchedKeyframeId)
        assertEquals(4, result.matchedLandmarksCount)
        assertEquals(TrackingQuality.TRACKING, result.targetPose?.trackingState)
    }

    @Test
    fun relocalization_failsOnMismatchedLandmarks_preservesLost() {
        val descA = ByteArray(16) { 0x00 }
        val descB = ByteArray(16) { 0xFF.toByte() } // Maximum Hamming distance (128 bits)

        val kfLandmarks = listOf(Landmark(1, 0f, 0f, 0f, 0f, descA))
        val keyframe = Keyframe(1, LocalPose.ORIGIN, kfLandmarks, 1000L)

        val currentLandmarks = listOf(Landmark(2, 0f, 0f, 0f, 0f, descB))
        val lostPose = LocalPose.ORIGIN.copy(trackingState = TrackingQuality.LOST)

        val result = relocalizer.attemptRelocalization(
            currentPose = lostPose,
            visibleLandmarks = currentLandmarks,
            keyframes = listOf(keyframe),
        )

        assertFalse(result.isSuccess)
    }
}
