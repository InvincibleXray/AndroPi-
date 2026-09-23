package com.minesafety.roboeye.net

import kotlinx.serialization.SerialName
import kotlinx.serialization.Serializable

@Serializable
data class LoginRequestDto(
    val username: String,
    val password: String,
)

@Serializable
data class TokenResponseDto(
    @SerialName("access_token") val accessToken: String,
    @SerialName("token_type") val tokenType: String = "bearer",
    val role: String,
    val username: String,
)

/**
 * One tracked object as it travels to the control room.
 *
 * Field names and nullability mirror `backend/app/schemas/perception.py::VisionRadarObject`
 * exactly. Pydantic ignores unknown keys, so a field that exists here and not there is not an
 * error — it is silently discarded, which is worse. Two such fields were removed rather than
 * added to the backend: `footprint_width_m` (the packet builder set it to the same expression
 * as `metric_width_m`, so it was a duplicate) and `estimated_mass_kg` (a bulk-density prior
 * applied to an assumed depth ratio applied to a monocular distance estimate — nothing
 * consumed it, and it is not a measurement anything should act on).
 */
@Serializable
data class VisionRadarObjectDto(
    val type: String,                       // VEHICLE | PERSON | OBSTACLE | ROAD_OBSTRUCTION | LARGE_OBJECT
    val label: String = "",
    val confidence: Float = 0.0f,
    @SerialName("estimated_distance_m") val estimatedDistanceM: Float? = null,
    @SerialName("relative_motion") val relativeMotion: String = "UNKNOWN", // APPROACHING | RECEDING | STATIONARY | UNKNOWN
    @SerialName("direction_deg") val directionDeg: Float? = null,           // Bearing within scan, degrees clockwise from ahead
    @SerialName("distance_is_measured") val distanceIsMeasured: Boolean = false, // Always false for vision radar!
    /**
     * This node's track id, so the same physical object keeps one identity across packets.
     * Without it the dashboard had to invent a key from list position, which renames every
     * object whenever the ordering changes.
     */
    @SerialName("object_id") val objectId: String? = null,
    /**
     * True when the tracker is carrying this object forward on prediction because it was not
     * matched in the latest frame. A coasting object is a belief, not an observation.
     */
    @SerialName("is_coasting") val isCoasting: Boolean = false,
    /**
     * Seconds since this object was last actually matched to a detection, measured on the
     * device. Distinct from the backend's `age_s`, which is transport age — how long since the
     * *packet* arrived. Only the node can know the first; only the backend can know the second,
     * and a fresh packet can carry an object last seen a second ago.
     */
    @SerialName("observation_age_s") val observationAgeS: Float? = null,
    @SerialName("metric_width_m") val metricWidthM: Float? = null,
    @SerialName("metric_height_m") val metricHeightM: Float? = null,
    @SerialName("is_on_road") val isOnRoad: Boolean? = null,
)

@Serializable
data class VisionRadarPacketDto(
    @SerialName("vehicle_id") val vehicleId: String,
    // Matches the backend schema's own default. RadarPacketBuilder always overrides this from
    // RadarConfig.nodeId, so it only ever surfaces for a hand-built or deserialized packet — but
    // the old default named a specific handset, which was a hardware claim this class cannot make.
    @SerialName("node_id") val nodeId: String = "VISION_RADAR",
    @SerialName("node_type") val nodeType: String = "VISION_RADAR",
    val timestamp: String? = null,          // ISO-8601 UTC
    @SerialName("scan_angle") val scanAngle: Float = 0.0f,
    @SerialName("scan_direction") val scanDirection: String = "FRONT",
    @SerialName("scan_priority") val scanPriority: String = "HIGH",
    @SerialName("scan_rate_dps") val scanRateDps: Float? = null,
    val visibility: Float? = null,
    @SerialName("perception_confidence") val perceptionConfidence: Float = 0.0f,
    val objects: List<VisionRadarObjectDto> = emptyList(),
    val simulated: Boolean = false,
    @SerialName("road_status") val roadStatus: String? = null,
    @SerialName("road_blockage_pct") val roadBlockagePct: Float? = null,
    @SerialName("traversable_width_m") val traversableWidthM: Float? = null,
    @SerialName("is_passable") val isPassable: Boolean? = null,
)

@Serializable
data class VisionRadarAckDto(
    val accepted: Boolean = true,
    @SerialName("vehicle_id") val vehicleId: String,
    @SerialName("objects_accepted") val objectsAccepted: Int = 0,
)

@Serializable
data class HealthInfoDto(
    val status: String? = null,
    val version: String? = null,
    val mode: String? = null,
)
