package com.minesafety.roboeye.net

import kotlinx.serialization.SerialName
import kotlinx.serialization.Serializable
import kotlinx.serialization.json.Json

/**
 * Wire types for the backend contract.
 *
 * Field names mirror `Robo Web App/backend/app/schemas/telemetry.py` and
 * `schemas/api.py` exactly; nothing here invents a field the backend does not accept.
 */

@Serializable
data class LoginRequest(val username: String, val password: String)

@Serializable
data class LoginResponse(
  @SerialName("access_token") val accessToken: String,
  @SerialName("token_type") val tokenType: String = "bearer",
  val role: String = "",
  val username: String = "",
)

/**
 * `GET /api/health` — unauthenticated liveness probe.
 *
 * [mode] echoes the backend's `TELEMETRY_PROVIDER`. Ingest is accepted in **either** mode, so
 * this is not a gate; it tells the operator what the *baseline* tick is. In `esp32` mode this
 * node is the only data source. In `simulation` mode the backend generates a tick and overlays
 * this node's real measurements onto it field by field — so any field the phone does not send
 * stays simulated, and the dashboard labels it as such.
 */
@Serializable
data class HealthInfo(
  val status: String = "",
  val version: String = "",
  val mode: String = "",
) {
  /**
   * True when the backend fills unprovided fields from its simulator rather than leaving them
   * unavailable. An unknown/absent `mode` is treated as simulated: assuming the safer
   * "these numbers may not be real" reading is the correct default.
   */
  val simulatedBaseline: Boolean get() = !mode.equals("esp32", ignoreCase = true)
}


/**
 * One semantic tracked object projected into the rover/world coordinate frame,
 * serialized from [com.minesafety.roboeye.nav.model.WorldObject].
 */
@Serializable
data class SemanticObjectTelemetryDto(
  @SerialName("track_id") val trackId: Int,
  val label: String = "",
  @SerialName("object_type") val objectType: String,
  val confidence: Float = 0.0f,
  @SerialName("timestamp_ms") val timestampMs: Long = 0L,
  @SerialName("rover_distance_m") val roverDistanceM: Float? = null,
  @SerialName("rover_bearing_deg") val roverBearingDeg: Float? = null,
  @SerialName("rover_x_m") val roverXM: Float? = null,
  @SerialName("rover_y_m") val roverYM: Float? = null,
  @SerialName("map_x_m") val mapXM: Float? = null,
  @SerialName("map_y_m") val mapYM: Float? = null,
  @SerialName("width_m") val widthM: Float? = null,
  @SerialName("height_m") val heightM: Float? = null,
  @SerialName("relative_motion") val relativeMotion: String = "UNKNOWN",
  val confirmed: Boolean = true,
  val coasting: Boolean = false,
  @SerialName("threat_score") val threatScore: Float? = null,
  @SerialName("distance_certainty") val distanceCertainty: String = "UNKNOWN",
)

/**
 * One telemetry frame, shaped as `TelemetryReading`.
 *
 * Every optional field is nullable and **omitted** when unknown (see [RoverJson]'s
 * `explicitNulls = false`) so the backend applies its own documented default rather than
 * receiving a number the phone invented. Concretely: with the camera stopped there is no
 * `visibility` key at all and `camera_available` is `false` — the same thing the reference
 * ESP32 firmware does when it has no camera.
 */
@Serializable
data class TelemetryFrame(
  @SerialName("vehicle_id") val vehicleId: String,
  /** ISO-8601 UTC, e.g. `2026-08-22T04:05:06.123Z`. */
  val timestamp: String,
  val speed: Float? = null,
  val battery: Float? = null,
  val visibility: Float? = null,
  @SerialName("ultrasonic_front") val ultrasonicFront: Float? = null,
  @SerialName("ultrasonic_left") val ultrasonicLeft: Float? = null,
  @SerialName("ultrasonic_right") val ultrasonicRight: Float? = null,
  @SerialName("ir_left") val irLeft: Int? = null,
  @SerialName("ir_center") val irCenter: Int? = null,
  @SerialName("ir_right") val irRight: Int? = null,
  @SerialName("imu_ax") val imuAx: Float? = null,
  @SerialName("imu_ay") val imuAy: Float? = null,
  @SerialName("imu_az") val imuAz: Float? = null,
  @SerialName("imu_pitch") val imuPitch: Float? = null,
  @SerialName("imu_roll") val imuRoll: Float? = null,
  val latitude: Double? = null,
  val longitude: Double? = null,
  @SerialName("camera_available") val cameraAvailable: Boolean = false,
  /**
   * Always `REAL_SENSOR`. The backend re-forces this server-side so a node cannot spoof
   * provenance; sending it keeps the intent explicit on the wire.
   */
  val source: String = SOURCE_REAL,
  @SerialName("semantic_objects") val semanticObjects: List<SemanticObjectTelemetryDto>? = null,
) {
  companion object {
    const val SOURCE_REAL = "REAL_SENSOR"
  }
}

/** Backend response to `POST /api/vehicles/{id}/telemetry`. */
@Serializable
data class IngestAck(
  val accepted: Boolean = false,
  @SerialName("vehicle_id") val vehicleId: String? = null,
  val commands: List<InboundFrame> = emptyList(),
)

/**
 * Any frame received from the backend on `WS /ws/ingest/{vehicle_id}`.
 *
 * The socket multiplexes four shapes over one lenient DTO: the `hello` handshake, an
 * `error` frame, command downlinks, and the `advisory` operating picture. The advisory's own
 * fields live in [AdvisoryFrame] — this type only needs to recognise it, because a frame is
 * routed on `type` before anything else is read.
 */
@Serializable
data class InboundFrame(
  val type: String? = null,
  /** Duplicate of [type] for command frames — the firmware-compatible key. */
  val command: String? = null,
  val value: Float? = null,
  val seq: Long? = null,
  val ts: String? = null,
  val error: String? = null,
  @SerialName("vehicle_id") val vehicleId: String? = null,
  @SerialName("tick_hz") val tickHz: Float? = null,
) {
  val kind: String get() = (type ?: command ?: "").uppercase()

  val isHello: Boolean get() = kind == "HELLO"

  val isError: Boolean get() = kind == "ERROR" || error != null

  /** True only for the seven command types the backend's `CommandType` enum defines. */
  val isCommand: Boolean get() = kind in COMMAND_TYPES

  /** The backend's per-tick operating picture. Carries no command — see [AdvisoryFrame]. */
  val isAdvisory: Boolean get() = kind == "ADVISORY"

  companion object {
    /** Mirrors `backend/app/core/constants.py::CommandType`. */
    val COMMAND_TYPES =
      setOf(
        "MOVE_FORWARD",
        "MOVE_BACKWARD",
        "TURN_LEFT",
        "TURN_RIGHT",
        "STOP",
        "SET_SPEED",
        "EMERGENCY_STOP",
      )
  }
}

/**
 * The backend's `{"type":"advisory", …}` downlink — shaped by `ws.py::advisory_frame()`.
 *
 * **This is the control room's recommendation, not a command and not a local computation.**
 * The phone renders what the backend decided so that the operator on the vehicle and the
 * operator in the control room are looking at the same words; a handset that derived its own
 * advice would eventually disagree with the dashboard, and in fog two disagreeing opinions are
 * worse than one. The node's local L1 mirror ([com.minesafety.roboeye.core.SafetyMirror]) is
 * untouched by this: that is a reflex, and it stays authoritative for its own e-stop.
 *
 * Every field is nullable because the backend trims the frame to what it has. A field the
 * backend did not send is rendered `NOT AVAILABLE`, never defaulted to a plausible number.
 */
@Serializable
data class AdvisoryFrame(
  @SerialName("vehicle_id") val vehicleId: String? = null,
  val ts: String? = null,
  val headline: String? = null,
  @SerialName("visibility_state") val visibilityState: String? = null,
  @SerialName("visibility_score") val visibilityScore: Float? = null,
  @SerialName("visibility_is_measured") val visibilityIsMeasured: Boolean? = null,
  @SerialName("visibility_basis") val visibilityBasis: String? = null,
  @SerialName("operating_mode") val operatingMode: String? = null,
  val guidance: String? = null,
  @SerialName("guidance_direction") val guidanceDirection: String? = null,
  @SerialName("collision_risk") val collisionRisk: String? = null,
  @SerialName("risk_level") val riskLevel: String? = null,
  @SerialName("risk_score") val riskScore: Float? = null,
  @SerialName("recommended_speed_mps") val recommendedSpeedMps: Float? = null,
  @SerialName("speed_limit_mps") val speedLimitMps: Float? = null,
  val overspeed: Boolean? = null,
  @SerialName("obstacle_distance_m") val obstacleDistanceM: Float? = null,
  val reason: String? = null,
  val authority: String? = null,
  val advisories: List<AdvisoryLine> = emptyList(),
  val detections: List<AdvisoryDetection> = emptyList(),
  @SerialName("telemetry_stale") val telemetryStale: Boolean? = null,
  @SerialName("comm_ok") val commOk: Boolean? = null,
) {
  /** True when the backend says this vehicle must stop. Display only — see the class note. */
  val isStop: Boolean get() = guidance?.uppercase() == "STOP"
}

/** One line of the backend's advisory list: what is wrong and what to do about it. */
@Serializable
data class AdvisoryLine(
  val severity: String? = null,
  val message: String? = null,
  val action: String? = null,
)

/**
 * One thing the backend's perception layer believes is present.
 *
 * `distance_m` is whatever the originating channel supplied — a measured ultrasonic range or a
 * camera-derived estimate — and `method` names which. The node displays both rather than
 * collapsing them into one number, because "measured 0.4 m" and "estimated 12 m" are not the
 * same claim.
 */
@Serializable
data class AdvisoryDetection(
  val kind: String? = null,
  val label: String? = null,
  @SerialName("distance_m") val distanceM: Float? = null,
  val confidence: Float? = null,
  val method: String? = null,
)

/** A validated command downlink, ready to forward to the ESP32. */
data class CommandFrame(
  val type: String,
  val value: Float?,
  val seq: Long?,
  val receivedAt: Long,
) {
  val isEmergency: Boolean get() = type == "EMERGENCY_STOP"

  val label: String
    get() = if (value != null) "$type ${String.format(java.util.Locale.US, "%.2f", value)}" else type

  companion object {
    fun from(frame: InboundFrame, now: Long): CommandFrame? =
      if (!frame.isCommand) null
      else CommandFrame(type = frame.kind, value = frame.value, seq = frame.seq, receivedAt = now)
  }
}

/**
 * Shared JSON configuration.
 *
 * * `explicitNulls = false` — unknown telemetry fields are omitted, not sent as `null`
 *   (Pydantic would reject `null` for the non-optional `speed`/`battery`/`visibility`).
 * * `encodeDefaults = true` — `camera_available` and `source` are always present.
 * * `ignoreUnknownKeys = true` — the backend may add response fields without breaking us.
 */
val RoverJson: Json = Json {
  explicitNulls = false
  encodeDefaults = true
  ignoreUnknownKeys = true
  isLenient = true
}
