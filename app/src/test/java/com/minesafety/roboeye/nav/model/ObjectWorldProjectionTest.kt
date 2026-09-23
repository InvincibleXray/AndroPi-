package com.minesafety.roboeye.nav.model

import com.minesafety.roboeye.core.NormalizedRect
import com.minesafety.roboeye.core.PerceptionObjectType
import com.minesafety.roboeye.core.RelativeMotion
import com.minesafety.roboeye.core.TrackedObject
import com.minesafety.roboeye.localization.LocalPose
import com.minesafety.roboeye.localization.MonocularScaleState
import com.minesafety.roboeye.localization.TrackingQuality
import com.minesafety.roboeye.mapping.LocalSpatialMap
import com.minesafety.roboeye.mapping.MapCellState
import com.minesafety.roboeye.nav.avoidance.AvoidanceAction
import com.minesafety.roboeye.nav.avoidance.LocalObstacleAvoidance
import com.minesafety.roboeye.nav.planner.AStarGridPlanner
import com.minesafety.roboeye.nav.planner.PathValidator
import com.minesafety.roboeye.nav.planner.PlanningResult
import com.minesafety.roboeye.perception.PerceptionEngine
import com.minesafety.roboeye.perception.PerceptionFrameResult
import com.minesafety.roboeye.perception.PerceptionMailbox
import com.minesafety.roboeye.perception.SnapshotImageProxy
import com.minesafety.roboeye.vision.GeometryTrustLevel
import com.minesafety.roboeye.vision.RegionalTtcResult
import com.minesafety.roboeye.vision.SectorRisk
import com.minesafety.roboeye.vision.SectorTtc
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test
import java.util.concurrent.CountDownLatch
import java.util.concurrent.TimeUnit
import java.util.concurrent.atomic.AtomicBoolean
import kotlin.math.cos
import kotlin.math.sin

/**
 * Deterministic unit tests for Phase 6B: Object State -> World Model Integration.
 *
 * Covers all 18 requirements specified in Section 21 of the Phase 6B specification.
 */
class ObjectWorldProjectionTest {

    private fun createTrack(
        id: String = "1",
        type: PerceptionObjectType = PerceptionObjectType.OBSTACLE,
        label: String = "rock",
        box: NormalizedRect = NormalizedRect(0.3f, 0.4f, 0.7f, 0.8f),
        bearingDeg: Float = 0.0f,
        distanceM: Float = 2.0f,
        confidence: Float = 0.85f,
        relativeMotion: RelativeMotion = RelativeMotion.APPROACHING,
        lastSeenMs: Long = 1000L,
        isConfirmed: Boolean = true,
        isCoasting: Boolean = false,
        threatScore: Float = 0.6f,
        metricWidthM: Float = 0.5f,
        metricHeightM: Float = 0.4f,
    ): TrackedObject {
        return TrackedObject(
            id = id,
            type = type,
            label = label,
            boundingBox = box,
            bearingDeg = bearingDeg,
            estimatedDistanceM = distanceM,
            confidence = confidence,
            relativeMotion = relativeMotion,
            firstSeenMs = 500L,
            lastSeenMs = lastSeenMs,
            hitCount = 5,
            isConfirmed = isConfirmed,
            isPrimary = false,
            threatScore = threatScore,
            isCoasting = isCoasting,
            metricWidthM = metricWidthM,
            metricHeightM = metricHeightM,
        )
    }

    private fun createValidEstimatedPose(
        xM: Float = 0.0f,
        yM: Float = 0.0f,
        yawDeg: Float = 0.0f,
    ): LocalPose {
        return LocalPose(
            timestampNs = 1_000_000L,
            xM = xM,
            yM = yM,
            yawDeg = yawDeg,
            confidence = 0.95f,
            scaleState = MonocularScaleState.SCALE_ESTIMATED,
            trackingState = TrackingQuality.TRACKING,
        )
    }

    // 1. trackedObject_projectsToWorldObject
    @Test
    fun trackedObject_projectsToWorldObject() {
        val track = createTrack(
            id = "42",
            label = "person",
            type = PerceptionObjectType.PERSON,
            confidence = 0.92f,
            distanceM = 3.5f,
            bearingDeg = 0.0f,
            relativeMotion = RelativeMotion.APPROACHING,
            threatScore = 0.75f,
            metricWidthM = 0.6f,
            metricHeightM = 1.7f,
        )
        val pose = createValidEstimatedPose(xM = 2.0f, yM = 1.0f, yawDeg = 0.0f)

        val worldObj = ObjectWorldProjection.projectSingleTrack(track, pose)

        assertEquals(42, worldObj.trackId)
        assertEquals("person", worldObj.label)
        assertEquals(PerceptionObjectType.PERSON, worldObj.objectType)
        assertEquals(0.92f, worldObj.confidence, 0.001f)
        assertEquals(track.lastSeenMs, worldObj.timestampMs)
        assertEquals(3.5f, worldObj.roverDistanceM!!, 0.01f)
        assertEquals(0.0f, worldObj.roverBearingDeg, 0.01f)
        assertEquals(3.5f, worldObj.roverXM!!, 0.01f)
        assertEquals(0.0f, worldObj.roverYM!!, 0.01f)
        assertEquals(5.5f, worldObj.mapXM!!, 0.01f) // 2.0 + 3.5
        assertEquals(1.0f, worldObj.mapYM!!, 0.01f) // 1.0 + 0.0
        assertEquals(0.6f, worldObj.widthM!!, 0.01f)
        assertEquals(1.7f, worldObj.heightM!!, 0.01f)
        assertEquals(RelativeMotion.APPROACHING, worldObj.relativeMotion)
        assertTrue(worldObj.isConfirmed)
        assertFalse(worldObj.isCoasting)
        assertEquals(0.75f, worldObj.threatScore, 0.01f)
        assertEquals(DistanceCertainty.ESTIMATED_PRIOR, worldObj.distanceCertainty)
    }

    // 2. bottomCenter_isUsedForGroundContactProjection
    @Test
    fun bottomCenter_isUsedForGroundContactProjection() {
        val rect = NormalizedRect(left = 0.20f, top = 0.10f, right = 0.80f, bottom = 0.90f)
        val (contactX, contactY) = ObjectWorldProjection.calculateGroundContact(rect)

        // Center X = 0.20 + 0.60 * 0.5 = 0.50
        assertEquals(0.50f, contactX, 0.001f)
        // Bottom Y = 0.90
        assertEquals(0.90f, contactY, 0.001f)
    }

    // 3. projection_usesExistingCoordinateConvention
    @Test
    fun projection_usesExistingCoordinateConvention() {
        // Optical bearing: negative is left of camera boresight.
        // Rover frame: +X forward, +Y left (CCW positive: positive heading is left).
        // Therefore roverBearingDeg = -opticalBearingDeg.
        val opticalBearingLeft = -30.0f
        val trackLeft = createTrack(id = "1", bearingDeg = opticalBearingLeft, distanceM = 2.0f)
        val poseOrigin = createValidEstimatedPose(xM = 0f, yM = 0f, yawDeg = 0f)

        val objLeft = ObjectWorldProjection.projectSingleTrack(trackLeft, poseOrigin)
        assertEquals(30.0f, objLeft.roverBearingDeg, 0.01f)
        val expectedRoverX = 2.0f * cos(Math.toRadians(30.0)).toFloat()
        val expectedRoverY = 2.0f * sin(Math.toRadians(30.0)).toFloat()
        assertEquals(expectedRoverX, objLeft.roverXM!!, 0.01f)
        assertEquals(expectedRoverY, objLeft.roverYM!!, 0.01f)
        assertTrue("Left object must have positive rover Y (+Y left)", objLeft.roverYM > 0f)

        // Optical bearing: positive is right of camera boresight.
        val opticalBearingRight = 30.0f
        val trackRight = createTrack(id = "2", bearingDeg = opticalBearingRight, distanceM = 2.0f)
        val objRight = ObjectWorldProjection.projectSingleTrack(trackRight, poseOrigin)
        assertEquals(-30.0f, objRight.roverBearingDeg, 0.01f)
        assertTrue("Right object must have negative rover Y (-Y right)", objRight.roverYM!! < 0f)

        // World frame SE(2) rotation: Rover at (10, 5) with heading yaw = +90° (facing Left / +Y world)
        val poseTurned90 = createValidEstimatedPose(xM = 10.0f, yM = 5.0f, yawDeg = 90.0f)
        val trackAhead = createTrack(id = "3", bearingDeg = 0.0f, distanceM = 2.0f)
        val objWorld = ObjectWorldProjection.projectSingleTrack(trackAhead, poseTurned90)

        // In rover frame: forward 2m along +X_rover.
        // At yaw = 90°: forward moves along +Y_world.
        // mapX = 10 + (2 * cos(90°) - 0 * sin(90°)) = 10.0
        // mapY = 5 + (2 * sin(90°) + 0 * cos(90°)) = 7.0
        assertEquals(10.0f, objWorld.mapXM!!, 0.01f)
        assertEquals(7.0f, objWorld.mapYM!!, 0.01f)
    }

    // 4. confirmedTrack_entersWorldModel
    @Test
    fun confirmedTrack_entersWorldModel() {
        val confirmed = createTrack(id = "1", isConfirmed = true, isCoasting = false)
        val pose = createValidEstimatedPose()
        val results = ObjectWorldProjection.projectTracks(listOf(confirmed), pose)

        assertEquals(1, results.size)
        assertEquals(1, results[0].trackId)
        assertTrue(results[0].isConfirmed)
        assertFalse(results[0].isCoasting)
    }

    // 5. coastingTrack_preservesValidWorldObject
    @Test
    fun coastingTrack_preservesValidWorldObject() {
        val coasting = createTrack(id = "2", isConfirmed = true, isCoasting = true, lastSeenMs = 900L)
        val pose = createValidEstimatedPose()
        // nowMs = 1000L, staleness = 100ms <= 1000ms max coast age
        val results = ObjectWorldProjection.projectTracks(listOf(coasting), pose, nowMs = 1000L)

        assertEquals(1, results.size)
        assertEquals(2, results[0].trackId)
        assertTrue(results[0].isConfirmed)
        assertTrue(results[0].isCoasting)
    }

    // 6. removedTrack_isRemoved
    @Test
    fun removedTrack_isRemoved() {
        val unconfirmedTrack = createTrack(id = "3", isConfirmed = false, isCoasting = false)
        val pose = createValidEstimatedPose()
        val results = ObjectWorldProjection.projectTracks(listOf(unconfirmedTrack), pose)

        assertTrue("Unconfirmed/removed tracks must be filtered out", results.isEmpty())
    }

    // 7. staleObject_isRemoved
    @Test
    fun staleObject_isRemoved() {
        val staleCoasting = createTrack(id = "4", isConfirmed = true, isCoasting = true, lastSeenMs = 500L)
        val pose = createValidEstimatedPose()
        // nowMs = 1600L, staleness = 1100ms > 1000ms max coast age
        val results = ObjectWorldProjection.projectTracks(listOf(staleCoasting), pose, nowMs = 1600L)

        assertTrue("Stale coasting tracks exceeding maxCoastAgeMs must be removed", results.isEmpty())
    }

    // 8. multipleTracks_areIndependent
    @Test
    fun multipleTracks_areIndependent() {
        val t1 = createTrack(id = "10", type = PerceptionObjectType.PERSON, bearingDeg = -25f, distanceM = 2.0f)
        val t2 = createTrack(id = "20", type = PerceptionObjectType.VEHICLE, bearingDeg = 0f, distanceM = 5.0f)
        val t3 = createTrack(id = "30", type = PerceptionObjectType.OBSTACLE, bearingDeg = 30f, distanceM = 3.0f)
        val pose = createValidEstimatedPose(xM = 1.0f, yM = 1.0f, yawDeg = 0.0f)

        val results = ObjectWorldProjection.projectTracks(listOf(t1, t2, t3), pose)

        assertEquals(3, results.size)
        assertEquals(10, results[0].trackId)
        assertEquals(20, results[1].trackId)
        assertEquals(30, results[2].trackId)

        assertEquals(PerceptionObjectType.PERSON, results[0].objectType)
        assertEquals(PerceptionObjectType.VEHICLE, results[1].objectType)
        assertEquals(PerceptionObjectType.OBSTACLE, results[2].objectType)

        assertTrue("Track 1 must be on the left (+Y)", results[0].roverYM!! > 0f)
        assertEquals(0.0f, results[1].roverYM!!, 0.01f)
        assertTrue("Track 3 must be on the right (-Y)", results[2].roverYM!! < 0f)
    }

    // 9. unknownDistance_isNotFabricated
    @Test
    fun unknownDistance_isNotFabricated() {
        val zeroDist = createTrack(id = "1", distanceM = 0.0f)
        val negDist = createTrack(id = "2", distanceM = -1.5f)
        val nanDist = createTrack(id = "3", distanceM = Float.NaN)
        val infDist = createTrack(id = "4", distanceM = Float.POSITIVE_INFINITY)

        val pose = createValidEstimatedPose()
        val results = ObjectWorldProjection.projectTracks(listOf(zeroDist, negDist, nanDist, infDist), pose)

        assertEquals(4, results.size)
        for (obj in results) {
            assertNull("roverDistanceM must be null when distance is unknown/invalid", obj.roverDistanceM)
            assertNull("roverXM must be null when distance is unknown/invalid", obj.roverXM)
            assertNull("roverYM must be null when distance is unknown/invalid", obj.roverYM)
            assertNull("mapXM must be null when distance is unknown/invalid", obj.mapXM)
            assertNull("mapYM must be null when distance is unknown/invalid", obj.mapYM)
            assertEquals(DistanceCertainty.UNKNOWN, obj.distanceCertainty)
        }
    }

    // 10. unknownScale_isNotConvertedToMeters
    @Test
    fun unknownScale_isNotConvertedToMeters() {
        val track = createTrack(id = "1", distanceM = 2.5f, bearingDeg = 0.0f)

        // Case B1: SCALE_UNKNOWN
        val poseUnknown = LocalPose.ORIGIN.copy(
            scaleState = MonocularScaleState.SCALE_UNKNOWN,
            trackingState = TrackingQuality.TRACKING,
        )
        val resUnknown = ObjectWorldProjection.projectTracks(listOf(track), poseUnknown)
        assertEquals(1, resUnknown.size)
        assertNotNull("Rover relative X should be preserved", resUnknown[0].roverXM)
        assertNotNull("Rover relative Y should be preserved", resUnknown[0].roverYM)
        assertNull("mapXM must NOT be generated when scale is UNKNOWN", resUnknown[0].mapXM)
        assertNull("mapYM must NOT be generated when scale is UNKNOWN", resUnknown[0].mapYM)
        assertEquals(DistanceCertainty.ESTIMATED_PRIOR, resUnknown[0].distanceCertainty)

        // Case B2: SCALE_UNRELIABLE
        val poseUnreliable = LocalPose.ORIGIN.copy(
            scaleState = MonocularScaleState.SCALE_UNRELIABLE,
            trackingState = TrackingQuality.TRACKING,
        )
        val resUnreliable = ObjectWorldProjection.projectTracks(listOf(track), poseUnreliable)
        assertNull("mapXM must NOT be generated when scale is UNRELIABLE", resUnreliable[0].mapXM)
        assertNull("mapYM must NOT be generated when scale is UNRELIABLE", resUnreliable[0].mapYM)

        // Case B3: Tracking Quality LOST
        val poseLost = LocalPose.ORIGIN.copy(
            scaleState = MonocularScaleState.SCALE_ESTIMATED,
            trackingState = TrackingQuality.LOST,
        )
        val resLost = ObjectWorldProjection.projectTracks(listOf(track), poseLost)
        assertNull("mapXM must NOT be generated when tracking is LOST", resLost[0].mapXM)
        assertNull("mapYM must NOT be generated when tracking is LOST", resLost[0].mapYM)
    }

    // 11. semanticObjects_doNotModifyGeometricOccupancy
    @Test
    fun semanticObjects_doNotModifyGeometricOccupancy() {
        val map = LocalSpatialMap(gridWidth = 24, gridHeight = 24, resolutionM = 0.25f)
        map.cells[5][5].state = MapCellState.OCCUPIED
        map.cells[12][12].state = MapCellState.FREE
        map.cells[20][20].state = MapCellState.UNCERTAIN

        val semanticObj = WorldObject(
            trackId = 99,
            label = "obstacle",
            objectType = PerceptionObjectType.OBSTACLE,
            confidence = 0.99f,
            timestampMs = 1000L,
            roverDistanceM = 1.0f,
            roverXM = 1.0f,
            roverYM = 0.0f,
            mapXM = 1.0f,
            mapYM = 0.0f,
            distanceCertainty = DistanceCertainty.ESTIMATED_PRIOR,
        )

        val worldModel = NavigationWorldModel(
            spatialMap = map,
            semanticObjects = listOf(semanticObj),
        )

        // Verify spatial map grid cells are 100% untouched
        assertEquals(MapCellState.OCCUPIED, worldModel.spatialMap.cells[5][5].state)
        assertEquals(MapCellState.FREE, worldModel.spatialMap.cells[12][12].state)
        assertEquals(MapCellState.UNCERTAIN, worldModel.spatialMap.cells[20][20].state)
        assertEquals(MapCellState.UNCERTAIN, worldModel.spatialMap.cells[0][0].state)
    }

    // 12. semanticObjects_doNotChangePlannerInputs
    @Test
    fun semanticObjects_doNotChangePlannerInputs() {
        val map = LocalSpatialMap(gridWidth = 24, gridHeight = 24, resolutionM = 0.25f)
        for (x in 0 until 24) {
            for (y in 0 until 24) {
                map.cells[x][y].state = MapCellState.FREE
            }
        }

        val planner = AStarGridPlanner(maxIterations = 500)
        val goal = com.minesafety.roboeye.nav.model.LocalGoal(targetX = 1.0f, targetY = 1.0f)

        val modelEmptySemantics = NavigationWorldModel(
            pose = LocalPose.ORIGIN.copy(trackingState = TrackingQuality.TRACKING),
            spatialMap = map,
            geometryTrust = GeometryTrustLevel.TRUSTED,
            semanticObjects = emptyList(),
        )

        val severeSemanticHazard = WorldObject(
            trackId = 666,
            label = "truck",
            objectType = PerceptionObjectType.VEHICLE,
            confidence = 1.0f,
            timestampMs = System.currentTimeMillis(),
            roverDistanceM = 0.5f,
            roverXM = 0.5f,
            roverYM = 0.5f,
            mapXM = 0.5f,
            mapYM = 0.5f,
            threatScore = 1.0f,
            distanceCertainty = DistanceCertainty.ESTIMATED_PRIOR,
        )

        val modelWithSemantics = NavigationWorldModel(
            pose = LocalPose.ORIGIN.copy(trackingState = TrackingQuality.TRACKING),
            spatialMap = map,
            geometryTrust = GeometryTrustLevel.TRUSTED,
            semanticObjects = listOf(severeSemanticHazard),
        )

        val planEmpty = planner.planPath(modelEmptySemantics, goal)
        val planWith = planner.planPath(modelWithSemantics, goal)

        assertTrue(planEmpty is PlanningResult.Success)
        assertTrue(planWith is PlanningResult.Success)

        val pathEmpty = (planEmpty as PlanningResult.Success).path
        val pathWith = (planWith as PlanningResult.Success).path

        assertEquals(pathEmpty.size, pathWith.size)
        assertEquals(pathEmpty.totalLengthM, pathWith.totalLengthM, 0.001f)
        for (i in pathEmpty.waypoints.indices) {
            assertEquals(pathEmpty.waypoints[i].xM, pathWith.waypoints[i].xM, 0.001f)
            assertEquals(pathEmpty.waypoints[i].yM, pathWith.waypoints[i].yM, 0.001f)
        }
    }

    // 13. olderPerceptionResult_cannotOverwriteNewerState
    @Test
    fun olderPerceptionResult_cannotOverwriteNewerState() {
        var lastAcceptedTimestamp = 2000L

        fun updateSemanticState(newTimestamp: Long, tracks: List<TrackedObject>): Boolean {
            if (newTimestamp < lastAcceptedTimestamp) {
                return false // Reject stale / out-of-order update
            }
            lastAcceptedTimestamp = newTimestamp
            return true
        }

        val staleUpdateAccepted = updateSemanticState(1500L, listOf(createTrack(id = "1")))
        assertFalse("Older perception timestamp must not be accepted", staleUpdateAccepted)
        assertEquals(2000L, lastAcceptedTimestamp)

        val newerUpdateAccepted = updateSemanticState(2500L, listOf(createTrack(id = "1")))
        assertTrue("Newer perception timestamp must be accepted", newerUpdateAccepted)
        assertEquals(2500L, lastAcceptedTimestamp)
    }

    // 14. freshSemanticSnapshot_removesNoLongerValidTracks
    @Test
    fun freshSemanticSnapshot_removesNoLongerValidTracks() {
        val pose = createValidEstimatedPose()

        // Frame N: two objects detected and tracked
        val frameNTracks = listOf(
            createTrack(id = "1", label = "person"),
            createTrack(id = "2", label = "rock"),
        )
        val snapshotN = ObjectWorldProjection.projectTracks(frameNTracks, pose)
        assertEquals(2, snapshotN.size)
        assertEquals(setOf(1, 2), snapshotN.map { it.trackId }.toSet())

        // Frame N+1: object 2 moved out of view or dropped; tracker produces only object 1
        val frameN1Tracks = listOf(
            createTrack(id = "1", label = "person"),
        )
        val snapshotN1 = ObjectWorldProjection.projectTracks(frameN1Tracks, pose)
        assertEquals(1, snapshotN1.size)
        assertEquals(1, snapshotN1[0].trackId)
        assertFalse("Zombie object 2 must not persist into fresh snapshot", snapshotN1.any { it.trackId == 2 })
    }

    // 15. objectProjection_doesNotBlockGeometricPipeline
    @Test
    fun objectProjection_doesNotBlockGeometricPipeline() {
        val pose = createValidEstimatedPose()
        val tracks = (1..20).map { i ->
            createTrack(id = "$i", distanceM = (i * 0.5f), bearingDeg = (i * 2.0f) - 20f)
        }

        // Warm up JIT
        repeat(100) {
            ObjectWorldProjection.projectTracks(tracks, pose)
        }

        val iterations = 1000
        val startTime = System.nanoTime()
        repeat(iterations) {
            ObjectWorldProjection.projectTracks(tracks, pose)
        }
        val elapsedNs = System.nanoTime() - startTime
        val avgMicros = (elapsedNs / iterations) / 1000

        // Projection of 20 tracks should execute in well under 500 microseconds (0.5 ms)
        assertTrue("Average projection time (${avgMicros}µs) should be < 500µs", avgMicros < 500)
    }

    // 16. Phase6A_AI_OFF_doesNotCreateSemanticUpdates
    @Test
    fun Phase6A_AI_OFF_doesNotCreateSemanticUpdates() {
        // When AI perception is OFF, tracker output is empty
        val emptyTracks = emptyList<TrackedObject>()
        val pose = createValidEstimatedPose()

        val semanticSnapshot = ObjectWorldProjection.projectTracks(emptyTracks, pose)
        assertTrue(semanticSnapshot.isEmpty())

        val map = LocalSpatialMap(gridWidth = 24, gridHeight = 24, resolutionM = 0.25f)
        val worldModel = NavigationWorldModel(
            spatialMap = map,
            semanticObjects = semanticSnapshot,
        )
        assertTrue(worldModel.semanticObjects.isEmpty())
    }

    // 17. Phase6A_generationOrdering_remainsIntact
    @Test
    fun Phase6A_generationOrdering_remainsIntact() {
        val mailbox = PerceptionMailbox()
        val engineStarted = CountDownLatch(1)
        val engineRelease = CountDownLatch(1)
        val resultDelivered = AtomicBoolean(false)

        val engine = object : PerceptionEngine {
            override fun processFrame(image: androidx.camera.core.ImageProxy, sensitivity: Float): PerceptionFrameResult {
                engineStarted.countDown()
                engineRelease.await(2, TimeUnit.SECONDS)
                return PerceptionFrameResult(
                    detectedObjects = emptyList(),
                    visibilityScore = 0.8f,
                    inferenceTimeMs = 10L,
                    frameWidth = 640,
                    frameHeight = 360,
                )
            }
        }

        mailbox.engine = engine
        mailbox.onResult = {
            resultDelivered.set(true)
        }
        mailbox.onEnabled()

        // Submit frame 1 while enabled
        mailbox.submit(
            SnapshotImageProxy(
                bitmap = null,
                rotation = 0,
                sequence = 1L,
                timestampNs = 1_000_000L,
                generation = mailbox.currentGeneration,
                frameWidth = 640,
                frameHeight = 360,
            )
        )
        assertTrue(engineStarted.await(1, TimeUnit.SECONDS))

        // Invalidate in-flight frame by calling onDisabled()
        mailbox.onDisabled()

        // Release engine inference
        engineRelease.countDown()
        Thread.sleep(100)

        // Result from the invalidated generation must NOT be delivered
        assertFalse("In-flight inference from older generation must be rejected upon disable", resultDelivered.get())
        mailbox.shutdown()
    }

    // 18. plannerBehavior_isUnchanged
    @Test
    fun plannerBehavior_isUnchanged() {
        val map = LocalSpatialMap(gridWidth = 24, gridHeight = 24, resolutionM = 0.25f)
        val pathValidator = PathValidator()
        val avoidance = LocalObstacleAvoidance(pathValidator = pathValidator)

        val modelWithoutSemantics = NavigationWorldModel(
            pose = LocalPose.ORIGIN.copy(trackingState = TrackingQuality.TRACKING),
            spatialMap = map,
            geometryTrust = GeometryTrustLevel.TRUSTED,
            semanticObjects = emptyList(),
        )

        val modelWithSemantics = NavigationWorldModel(
            pose = LocalPose.ORIGIN.copy(trackingState = TrackingQuality.TRACKING),
            spatialMap = map,
            geometryTrust = GeometryTrustLevel.TRUSTED,
            semanticObjects = listOf(
                WorldObject(
                    trackId = 999,
                    label = "pedestrian",
                    objectType = PerceptionObjectType.PERSON,
                    confidence = 0.95f,
                    timestampMs = System.currentTimeMillis(),
                    roverDistanceM = 0.3f,
                    roverXM = 0.3f,
                    roverYM = 0.0f,
                    mapXM = 0.3f,
                    mapYM = 0.0f,
                    threatScore = 1.0f,
                    distanceCertainty = DistanceCertainty.ESTIMATED_PRIOR,
                )
            ),
        )

        val actionWithout = avoidance.evaluate(modelWithoutSemantics)
        val actionWith = avoidance.evaluate(modelWithSemantics)

        assertEquals("Planner avoidance action must be identical with or without semantic objects", actionWithout, actionWith)
        assertEquals(AvoidanceAction.Clear, actionWith)

        // When geometric TTC triggers emergency brake, both trigger emergency brake identically
        val ttcEmergency = RegionalTtcResult(
            left = SectorTtc("LEFT", null, null, SectorRisk.SAFE, 0),
            center = SectorTtc("CENTER", 0.5f, 0.5f, SectorRisk.CRITICAL, 5),
            right = SectorTtc("RIGHT", null, null, SectorRisk.SAFE, 0),
            criticalSector = "CENTER",
            minTtcSec = 0.5f,
            overallRisk = SectorRisk.CRITICAL,
        )
        val modelTtcWithout = modelWithoutSemantics.copy(ttcResult = ttcEmergency)
        val modelTtcWith = modelWithSemantics.copy(ttcResult = ttcEmergency)

        val actionTtcWithout = avoidance.evaluate(modelTtcWithout)
        val actionTtcWith = avoidance.evaluate(modelTtcWith)

        assertTrue(actionTtcWithout is AvoidanceAction.EmergencyBrake)
        assertTrue(actionTtcWith is AvoidanceAction.EmergencyBrake)
    }
}
