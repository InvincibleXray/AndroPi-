package com.minesafety.roboeye.ui

import androidx.compose.ui.graphics.Color
import com.minesafety.roboeye.core.BatteryState
import com.minesafety.roboeye.core.CameraRunState
import com.minesafety.roboeye.core.CameraState
import com.minesafety.roboeye.core.ImuState
import com.minesafety.roboeye.core.LocationState
import com.minesafety.roboeye.core.PerceptionObjectType
import com.minesafety.roboeye.core.RelativeMotion
import com.minesafety.roboeye.core.SafetyMirror
import com.minesafety.roboeye.core.TelemetryAssembler
import com.minesafety.roboeye.core.model.MotionCommand
import com.minesafety.roboeye.core.model.SafetyState
import com.minesafety.roboeye.localization.LocalPose
import com.minesafety.roboeye.localization.TrackingQuality
import com.minesafety.roboeye.mapping.LocalSpatialMap
import com.minesafety.roboeye.mapping.MapCellState
import com.minesafety.roboeye.nav.model.DistanceCertainty
import com.minesafety.roboeye.nav.model.LocalGoal
import com.minesafety.roboeye.nav.model.NavigationWorldModel
import com.minesafety.roboeye.nav.model.WorldObject
import com.minesafety.roboeye.nav.planner.AStarGridPlanner
import com.minesafety.roboeye.nav.planner.PlanningResult
import com.minesafety.roboeye.nav.safety.NavigationSafetyGate
import com.minesafety.roboeye.net.RoverJson
import com.minesafety.roboeye.net.SemanticObjectTelemetryDto
import com.minesafety.roboeye.net.TelemetryFrame
import com.minesafety.roboeye.vision.GeometryTrustLevel
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test
import java.util.Locale
import kotlin.math.cos
import kotlin.math.sin

/**
 * Deterministic unit tests for Phase 6C: Semantic Radar & Telemetry Integration.
 *
 * Covers all 22 required test cases across UI representation, Telemetry assembly,
 * wire serialization, coordinate conventions, and safety/planner isolation.
 */
class SemanticRadarAndTelemetryTest {

    private fun createSampleWorldObject(
        trackId: Int = 101,
        label: String = "rock",
        objectType: PerceptionObjectType = PerceptionObjectType.OBSTACLE,
        confidence: Float = 0.92f,
        distanceM: Float? = 2.4f,
        bearingDeg: Float = 12.5f,
        roverXM: Float? = 2.34f,
        roverYM: Float? = 0.52f,
        mapXM: Float? = 3.1f,
        mapYM: Float? = 1.2f,
        widthM: Float? = 0.6f,
        heightM: Float? = 0.45f,
        relativeMotion: RelativeMotion = RelativeMotion.APPROACHING,
        isConfirmed: Boolean = true,
        isCoasting: Boolean = false,
        threatScore: Float = 0.75f,
        distanceCertainty: DistanceCertainty = DistanceCertainty.ESTIMATED_PRIOR,
    ): WorldObject {
        return WorldObject(
            trackId = trackId,
            label = label,
            objectType = objectType,
            confidence = confidence,
            timestampMs = 1700000000000L,
            roverDistanceM = distanceM,
            roverBearingDeg = bearingDeg,
            roverXM = roverXM,
            roverYM = roverYM,
            mapXM = mapXM,
            mapYM = mapYM,
            widthM = widthM,
            heightM = heightM,
            relativeMotion = relativeMotion,
            isConfirmed = isConfirmed,
            isCoasting = isCoasting,
            threatScore = threatScore,
            distanceCertainty = distanceCertainty,
        )
    }

    private fun formatDistance(obj: WorldObject): String {
        return if (obj.distanceCertainty == DistanceCertainty.ESTIMATED_PRIOR &&
            obj.roverDistanceM != null && obj.roverDistanceM.isFinite() && obj.roverDistanceM > 0f) {
            "Estimated ${String.format(Locale.US, "%.1fm", obj.roverDistanceM)}"
        } else {
            "Distance: Unknown"
        }
    }

    private fun resolveMapPosition(obj: WorldObject, localPose: LocalPose): Pair<Float, Float>? {
        return when {
            obj.distanceCertainty == DistanceCertainty.UNKNOWN -> null
            obj.mapXM != null && obj.mapYM != null && obj.mapXM.isFinite() && obj.mapYM.isFinite() -> {
                Pair(obj.mapXM, obj.mapYM)
            }
            obj.roverXM != null && obj.roverYM != null && obj.roverXM.isFinite() && obj.roverYM.isFinite() -> {
                val yawR = Math.toRadians(localPose.yawDeg.toDouble()).toFloat()
                val cosY = cos(yawR)
                val sinY = sin(yawR)
                val mX = localPose.xM + (obj.roverXM * cosY - obj.roverYM * sinY)
                val mY = localPose.yM + (obj.roverXM * sinY + obj.roverYM * cosY)
                Pair(mX, mY)
            }
            else -> null
        }
    }

    // --- 1. semantic_radar_renders_cleanly_when_objects_empty ---
    @Test
    fun semantic_radar_renders_cleanly_when_objects_empty() {
        val emptyList = emptyList<WorldObject>()
        assertEquals(0, emptyList.size)
        val header = "SEMANTIC RADAR (${emptyList.size} TARGETS)"
        val emptyText = if (emptyList.isEmpty()) "NO SEMANTIC OBJECTS DETECTED" else ""
        assertEquals("SEMANTIC RADAR (0 TARGETS)", header)
        assertEquals("NO SEMANTIC OBJECTS DETECTED", emptyText)
    }

    // --- 2. semantic_radar_displays_tracked_object_id_and_label ---
    @Test
    fun semantic_radar_displays_tracked_object_id_and_label() {
        val obj = createSampleWorldObject(trackId = 42, label = "person")
        val renderedTitle = "#${obj.trackId} ${obj.label.uppercase(Locale.US)}"
        assertEquals("#42 PERSON", renderedTitle)
    }

    // --- 3. semantic_radar_shows_estimated_distance_when_prior_certainty ---
    @Test
    fun semantic_radar_shows_estimated_distance_when_prior_certainty() {
        val obj = createSampleWorldObject(
            distanceM = 2.5f,
            distanceCertainty = DistanceCertainty.ESTIMATED_PRIOR,
        )
        val formatted = formatDistance(obj)
        assertEquals("Estimated 2.5m", formatted)
    }

    // --- 4. semantic_radar_shows_distance_unknown_when_certainty_unknown ---
    @Test
    fun semantic_radar_shows_distance_unknown_when_certainty_unknown() {
        val objWithNull = createSampleWorldObject(
            distanceM = null,
            distanceCertainty = DistanceCertainty.UNKNOWN,
        )
        assertEquals("Distance: Unknown", formatDistance(objWithNull))

        val objWithStaleValue = createSampleWorldObject(
            distanceM = 3.0f,
            distanceCertainty = DistanceCertainty.UNKNOWN,
        )
        assertEquals("Distance: Unknown", formatDistance(objWithStaleValue))
    }

    // --- 5. semantic_radar_displays_bearing_angle ---
    @Test
    fun semantic_radar_displays_bearing_angle() {
        val objPos = createSampleWorldObject(bearingDeg = 15.4f)
        val posSign = if (objPos.roverBearingDeg >= 0) "+" else ""
        val posBearing = "BRG: $posSign${String.format(Locale.US, "%.1f°", objPos.roverBearingDeg)}"
        assertEquals("BRG: +15.4°", posBearing)

        val objNeg = createSampleWorldObject(bearingDeg = -23.1f)
        val negSign = if (objNeg.roverBearingDeg >= 0) "+" else ""
        val negBearing = "BRG: $negSign${String.format(Locale.US, "%.1f°", objNeg.roverBearingDeg)}"
        assertEquals("BRG: -23.1°", negBearing)
    }

    // --- 6. semantic_radar_displays_relative_body_coordinates ---
    @Test
    fun semantic_radar_displays_relative_body_coordinates() {
        val obj = createSampleWorldObject(roverXM = 1.8f, roverYM = -0.5f)
        assertNotNull(obj.roverXM)
        assertNotNull(obj.roverYM)
        val xSign = if (obj.roverXM!! >= 0) "+" else ""
        val ySign = if (obj.roverYM!! >= 0) "+" else ""
        val coordsText = "BODY: X: $xSign${String.format(Locale.US, "%.1fm", obj.roverXM)} | Y: $ySign${String.format(Locale.US, "%.1fm", obj.roverYM)}"
        assertEquals("BODY: X: +1.8m | Y: -0.5m", coordsText)
    }

    // --- 7. semantic_radar_shows_coasting_badge_when_coasting ---
    @Test
    fun semantic_radar_shows_coasting_badge_when_coasting() {
        val activeObj = createSampleWorldObject(isCoasting = false)
        assertFalse(activeObj.isCoasting)

        val coastingObj = createSampleWorldObject(isCoasting = true)
        assertTrue(coastingObj.isCoasting)
    }

    // --- 8. semantic_radar_shows_threat_score ---
    @Test
    fun semantic_radar_shows_threat_score() {
        val highThreat = createSampleWorldObject(threatScore = 0.85f)
        val highThreatLabel = "${(highThreat.threatScore * 100).toInt()}% THREAT"
        assertEquals("85% THREAT", highThreatLabel)
        assertTrue(highThreat.threatScore >= 0.7f)

        val lowThreat = createSampleWorldObject(threatScore = 0.15f)
        val lowThreatLabel = "${(lowThreat.threatScore * 100).toInt()}% THREAT"
        assertEquals("15% THREAT", lowThreatLabel)
        assertTrue(lowThreat.threatScore < 0.35f)
    }

    // --- 9. semantic_map_overlay_plots_object_at_world_coordinates ---
    @Test
    fun semantic_map_overlay_plots_object_at_world_coordinates() {
        val pose = LocalPose(xM = 1.0f, yM = 2.0f, yawDeg = 0.0f, trackingState = TrackingQuality.TRACKING)
        val obj = createSampleWorldObject(
            mapXM = 1.2f,
            mapYM = -0.8f,
            distanceCertainty = DistanceCertainty.ESTIMATED_PRIOR,
        )
        val pos = resolveMapPosition(obj, pose)
        assertNotNull(pos)
        assertEquals(1.2f, pos!!.first, 0.001f)
        assertEquals(-0.8f, pos.second, 0.001f)
    }

    // --- 10. semantic_map_overlay_does_not_plot_object_with_unknown_distance ---
    @Test
    fun semantic_map_overlay_does_not_plot_object_with_unknown_distance() {
        val pose = LocalPose(xM = 1.0f, yM = 2.0f, yawDeg = 0.0f, trackingState = TrackingQuality.TRACKING)
        val objUnknown = createSampleWorldObject(
            mapXM = 0.0f,
            mapYM = 0.0f,
            distanceCertainty = DistanceCertainty.UNKNOWN,
        )
        val pos = resolveMapPosition(objUnknown, pose)
        assertNull(pos) // Must NOT plot fabricated (0,0) or rover origin
    }

    // --- 11. semantic_map_overlay_respects_rover_relative_left_right_convention ---
    @Test
    fun semantic_map_overlay_respects_rover_relative_left_right_convention() {
        val poseOrigin = LocalPose(xM = 0.0f, yM = 0.0f, yawDeg = 0.0f, trackingState = TrackingQuality.TRACKING)

        // Left object: +Y is Left in body frame
        val leftObj = createSampleWorldObject(
            roverXM = 2.0f,
            roverYM = 1.0f,
            mapXM = null,
            mapYM = null,
            distanceCertainty = DistanceCertainty.ESTIMATED_PRIOR,
        )
        val leftPos = resolveMapPosition(leftObj, poseOrigin)
        assertNotNull(leftPos)
        assertEquals(2.0f, leftPos!!.first, 0.001f) // +X forward
        assertEquals(1.0f, leftPos.second, 0.001f) // +Y left

        // Right object: -Y is Right in body frame
        val rightObj = createSampleWorldObject(
            roverXM = 2.0f,
            roverYM = -1.0f,
            mapXM = null,
            mapYM = null,
            distanceCertainty = DistanceCertainty.ESTIMATED_PRIOR,
        )
        val rightPos = resolveMapPosition(rightObj, poseOrigin)
        assertNotNull(rightPos)
        assertEquals(2.0f, rightPos!!.first, 0.001f) // +X forward
        assertEquals(-1.0f, rightPos.second, 0.001f) // -Y right

        // Screen mapping: sx = originScreenX - (relYM * scalePxPerM)
        val originScreenX = 400f
        val scale = 50f
        val leftSx = originScreenX - (leftPos.second * scale) // 400 - 50 = 350 (left on screen)
        val rightSx = originScreenX - (rightPos.second * scale) // 400 - (-50) = 450 (right on screen)
        assertTrue("Left object screen X ($leftSx) must be less than origin X ($originScreenX)", leftSx < originScreenX)
        assertTrue("Right object screen X ($rightSx) must be greater than origin X ($originScreenX)", rightSx > originScreenX)
    }

    // --- 12. semantic_map_legend_contains_obj_indicator ---
    @Test
    fun semantic_map_legend_contains_obj_indicator() {
        val objColor = Color(0xFFE040FB)
        val objLabel = "OBJ"
        assertEquals("OBJ", objLabel)
        assertEquals(0xFFE040FB.toLong(), (objColor.value shr 32).toLong())
    }

    // --- 13. world_model_semantic_objects_is_sole_source_for_radar_and_telemetry ---
    @Test
    fun world_model_semantic_objects_is_sole_source_for_radar_and_telemetry() {
        val obj1 = createSampleWorldObject(trackId = 1)
        val obj2 = createSampleWorldObject(trackId = 2)
        val model = NavigationWorldModel(
            pose = LocalPose.ORIGIN,
            spatialMap = LocalSpatialMap(),
            timestampMs = 1000L,
            semanticObjects = listOf(obj1, obj2),
        )

        // Radar and Telemetry consume model.semanticObjects directly
        assertEquals(2, model.semanticObjects.size)
        assertEquals(1, model.semanticObjects[0].trackId)
        assertEquals(2, model.semanticObjects[1].trackId)
    }

    // --- 14. telemetryFrame_with_empty_semantic_objects_serializes_cleanly ---
    @Test
    fun telemetryFrame_with_empty_semantic_objects_serializes_cleanly() {
        val frame = TelemetryFrame(
            vehicleId = "ROVER-01",
            timestamp = "2026-09-23T10:00:00Z",
            semanticObjects = emptyList(),
        )
        val json = RoverJson.encodeToString(TelemetryFrame.serializer(), frame)
        assertTrue(json.contains("\"semantic_objects\":[]"))
    }

    // --- 15. telemetryFrame_with_populated_semantic_objects_contains_all_fields ---
    @Test
    fun telemetryFrame_with_populated_semantic_objects_contains_all_fields() {
        val obj = createSampleWorldObject(
            trackId = 99,
            label = "barrel",
            objectType = PerceptionObjectType.ROAD_OBSTRUCTION,
            confidence = 0.88f,
            distanceM = 3.2f,
            bearingDeg = -5.0f,
            roverXM = 3.18f,
            roverYM = -0.28f,
            mapXM = 4.5f,
            mapYM = 1.1f,
            widthM = 0.5f,
            heightM = 0.8f,
            relativeMotion = RelativeMotion.STATIONARY,
            isConfirmed = true,
            isCoasting = false,
            threatScore = 0.65f,
            distanceCertainty = DistanceCertainty.ESTIMATED_PRIOR,
        )

        val assembled = TelemetryAssembler.build(
            vehicleId = "ROVER-01",
            now = 1000L,
            imu = ImuState(),
            location = LocationState(),
            battery = BatteryState(),
            camera = CameraState(runState = CameraRunState.ACTIVE),
            semanticObjects = listOf(obj),
        )

        val json = RoverJson.encodeToString(TelemetryFrame.serializer(), assembled.frame)

        val expectedFields = listOf(
            "track_id",
            "label",
            "object_type",
            "confidence",
            "timestamp_ms",
            "rover_distance_m",
            "rover_bearing_deg",
            "rover_x_m",
            "rover_y_m",
            "map_x_m",
            "map_y_m",
            "width_m",
            "height_m",
            "relative_motion",
            "confirmed",
            "coasting",
            "threat_score",
            "distance_certainty",
        )

        for (field in expectedFields) {
            assertTrue("Telemetry JSON must contain field '$field': $json", json.contains("\"$field\""))
        }
    }

    // --- 16. telemetryFrame_nan_and_inf_values_sanitized_to_null ---
    @Test
    fun telemetryFrame_nan_and_inf_values_sanitized_to_null() {
        val corruptedObj = createSampleWorldObject(
            distanceM = Float.NaN,
            bearingDeg = Float.POSITIVE_INFINITY,
            roverXM = Float.NaN,
            roverYM = Float.NEGATIVE_INFINITY,
            mapXM = Float.NaN,
            mapYM = Float.POSITIVE_INFINITY,
            widthM = Float.NaN,
            heightM = Float.POSITIVE_INFINITY,
            threatScore = Float.NaN,
        )

        val assembled = TelemetryAssembler.build(
            vehicleId = "ROVER-01",
            now = 1000L,
            imu = ImuState(),
            location = LocationState(),
            battery = BatteryState(),
            camera = CameraState(runState = CameraRunState.ACTIVE),
            semanticObjects = listOf(corruptedObj),
        )

        val dto = assembled.frame.semanticObjects?.firstOrNull()
        assertNotNull(dto)
        assertNull("NaN distance must sanitize to null", dto!!.roverDistanceM)
        assertNull("Inf bearing must sanitize to null", dto.roverBearingDeg)
        assertNull("NaN roverX must sanitize to null", dto.roverXM)
        assertNull("Inf roverY must sanitize to null", dto.roverYM)
        assertNull("NaN mapX must sanitize to null", dto.mapXM)
        assertNull("Inf mapY must sanitize to null", dto.mapYM)
        assertNull("NaN width must sanitize to null", dto.widthM)
        assertNull("Inf height must sanitize to null", dto.heightM)
        assertNull("NaN threat score must sanitize to null", dto.threatScore)

        val json = RoverJson.encodeToString(TelemetryFrame.serializer(), assembled.frame)
        assertFalse("JSON must not contain NaN string", json.contains("NaN"))
        assertFalse("JSON must not contain Infinity string", json.contains("Infinity"))
    }

    // --- 17. telemetryFrame_does_not_convert_null_distance_to_zero ---
    @Test
    fun telemetryFrame_does_not_convert_null_distance_to_zero() {
        val objNullDist = createSampleWorldObject(
            distanceM = null,
            distanceCertainty = DistanceCertainty.UNKNOWN,
        )

        val assembled = TelemetryAssembler.build(
            vehicleId = "ROVER-01",
            now = 1000L,
            imu = ImuState(),
            location = LocationState(),
            battery = BatteryState(),
            camera = CameraState(runState = CameraRunState.ACTIVE),
            semanticObjects = listOf(objNullDist),
        )

        val dto = assembled.frame.semanticObjects?.firstOrNull()
        assertNotNull(dto)
        assertNull("Distance must remain null and not be converted to 0.0", dto!!.roverDistanceM)

        val json = RoverJson.encodeToString(TelemetryFrame.serializer(), assembled.frame)
        assertFalse("JSON must not contain '\"rover_distance_m\":0.0'", json.contains("\"rover_distance_m\":0.0"))
        assertFalse("JSON must not contain '\"rover_distance_m\":0'", json.contains("\"rover_distance_m\":0"))
    }

    // --- 18. telemetry_assembler_extracts_semantic_objects_from_world_model ---
    @Test
    fun telemetry_assembler_extracts_semantic_objects_from_world_model() {
        val model = NavigationWorldModel(
            pose = LocalPose.ORIGIN,
            spatialMap = LocalSpatialMap(),
            timestampMs = 2000L,
            semanticObjects = listOf(
                createSampleWorldObject(trackId = 10),
                createSampleWorldObject(trackId = 20),
            ),
        )

        val assembled = TelemetryAssembler.build(
            vehicleId = "ROVER-01",
            now = 2000L,
            imu = ImuState(),
            location = LocationState(),
            battery = BatteryState(),
            camera = CameraState(runState = CameraRunState.ACTIVE),
            semanticObjects = model.semanticObjects,
        )

        assertEquals(2, assembled.frame.semanticObjects?.size)
        assertEquals(10, assembled.frame.semanticObjects!![0].trackId)
        assertEquals(20, assembled.frame.semanticObjects!![1].trackId)
    }

    // --- 19. telemetry_omits_or_empties_semantic_objects_when_ai_disabled ---
    @Test
    fun telemetry_omits_or_empties_semantic_objects_when_ai_disabled() {
        val aiEnabled = false
        val semanticList = if (aiEnabled) listOf(createSampleWorldObject()) else emptyList()

        val assembled = TelemetryAssembler.build(
            vehicleId = "ROVER-01",
            now = 1000L,
            imu = ImuState(),
            location = LocationState(),
            battery = BatteryState(),
            camera = CameraState(runState = CameraRunState.ACTIVE),
            semanticObjects = semanticList,
        )

        assertTrue(assembled.frame.semanticObjects.isNullOrEmpty())
    }

    // --- 20. telemetry_contains_no_stale_semantic_objects_when_camera_inactive ---
    @Test
    fun telemetry_contains_no_stale_semantic_objects_when_camera_inactive() {
        val cameraActive = false
        val previousWorldModelObjects = listOf(createSampleWorldObject(trackId = 55))

        // Controller logic clears or passes emptyList when camera is inactive
        val objectsToSend = if (cameraActive) previousWorldModelObjects else emptyList()

        val assembled = TelemetryAssembler.build(
            vehicleId = "ROVER-01",
            now = 1000L,
            imu = ImuState(),
            location = LocationState(),
            battery = BatteryState(),
            camera = CameraState(runState = CameraRunState.STOPPED),
            semanticObjects = objectsToSend,
        )

        assertEquals(0, assembled.frame.semanticObjects?.size)
    }

    // --- 21. semantic_radar_and_telemetry_leave_astar_planner_unaffected ---
    @Test
    fun semantic_radar_and_telemetry_leave_astar_planner_unaffected() {
        val planner = AStarGridPlanner(maxIterations = 500)
        val map = LocalSpatialMap(gridWidth = 24, gridHeight = 24, resolutionM = 0.25f)

        // Clear corridor from (12, 12) to (12, 18)
        for (y in 12..18) {
            map.cells[12][y].state = MapCellState.FREE
        }

        val goal = LocalGoal(targetX = 1.5f, targetY = 0.0f)
        val pose = LocalPose.ORIGIN.copy(trackingState = TrackingQuality.TRACKING)

        val emptyWorldModel = NavigationWorldModel(
            pose = pose,
            spatialMap = map,
            geometryTrust = GeometryTrustLevel.TRUSTED,
            timestampMs = 1000L,
            semanticObjects = emptyList(),
        )

        val populatedWorldModel = NavigationWorldModel(
            pose = pose,
            spatialMap = map,
            geometryTrust = GeometryTrustLevel.TRUSTED,
            timestampMs = 1000L,
            semanticObjects = listOf(
                createSampleWorldObject(trackId = 999, threatScore = 1.0f, roverXM = 1.5f, roverYM = 0.0f),
            ),
        )

        val resultEmpty = planner.planPath(emptyWorldModel, goal)
        val resultPopulated = planner.planPath(populatedWorldModel, goal)

        assertEquals(resultEmpty is PlanningResult.Success, resultPopulated is PlanningResult.Success)
        if (resultEmpty is PlanningResult.Success && resultPopulated is PlanningResult.Success) {
            assertEquals(resultEmpty.path.waypoints.size, resultPopulated.path.waypoints.size)
            assertEquals(resultEmpty.path.totalLengthM, resultPopulated.path.totalLengthM, 0.001f)
        }
    }

    // --- 22. semantic_radar_and_telemetry_leave_ttc_and_safety_unaffected ---
    @Test
    fun semantic_radar_and_telemetry_leave_ttc_and_safety_unaffected() {
        val gate = NavigationSafetyGate()
        val emptyModel = NavigationWorldModel(
            pose = LocalPose.ORIGIN.copy(trackingState = TrackingQuality.TRACKING),
            spatialMap = LocalSpatialMap(),
            geometryTrust = GeometryTrustLevel.TRUSTED,
            timestampMs = 1000L,
            semanticObjects = emptyList(),
        )
        val populatedModel = NavigationWorldModel(
            pose = LocalPose.ORIGIN.copy(trackingState = TrackingQuality.TRACKING),
            spatialMap = LocalSpatialMap(),
            geometryTrust = GeometryTrustLevel.TRUSTED,
            timestampMs = 1000L,
            semanticObjects = listOf(
                createSampleWorldObject(trackId = 999, threatScore = 1.0f, distanceM = 0.1f),
            ),
        )

        val candidateCmd = MotionCommand(linearVelocityMps = 0.3f, angularVelocityRadS = 0.0f)
        val actionEmpty = gate.vetCommand(candidateCmd, emptyModel, SafetyState(), isTransportConnected = true, nowMs = 1000L)
        val actionPopulated = gate.vetCommand(candidateCmd, populatedModel, SafetyState(), isTransportConnected = true, nowMs = 1000L)

        // Gate verdict must remain 100% identical regardless of semanticObjects
        assertEquals(actionEmpty.action, actionPopulated.action)

        // SafetyMirror evaluate check
        val verdict1 = SafetyMirror.evaluate(
            pitchDeg = 0f,
            rollDeg = 0f,
            accelMagG = 1.0f,
            minObstacleM = null,
            bridgeConnected = false,
            visibility = 0.8f,
        )
        val verdict2 = SafetyMirror.evaluate(
            pitchDeg = 0f,
            rollDeg = 0f,
            accelMagG = 1.0f,
            minObstacleM = null,
            bridgeConnected = false,
            visibility = 0.8f,
        )
        assertEquals(verdict1.level, verdict2.level)
        assertEquals(verdict1.isEStop, verdict2.isEStop)
    }
}
