package com.minesafety.roboeye.core

import com.minesafety.roboeye.bridge.BridgeTelemetry
import com.minesafety.roboeye.bridge.LinkStatus
import com.minesafety.roboeye.net.AdvisoryFrame
import com.minesafety.roboeye.net.CommandFrame
import com.minesafety.roboeye.sensors.VisibilityResult

/**
 * Whether a given sensor can be read at all.
 *
 * The Rover Node never substitutes a placeholder number for a missing sensor — the UI
 * renders `NOT AVAILABLE` and the telemetry frame omits the field (or sends the schema
 * default) so the backend's provenance stays honest.
 */
enum class Availability {
  /** Hardware present and delivering samples. */
  AVAILABLE,

  /** Hardware present, no sample yet (e.g. GPS has no fix). */
  WAITING,

  /** No such sensor on this device. */
  UNAVAILABLE,

  /** Sensor exists but the runtime permission was not granted. */
  NO_PERMISSION;

  val isUsable: Boolean get() = this == AVAILABLE
}

// ---------------------------------------------------------------------------
// Per-subsystem state
// ---------------------------------------------------------------------------

data class ImuReading(
  /** Linear (gravity-removed) acceleration in **g**. */
  val axG: Float = 0f,
  val ayG: Float = 0f,
  val azG: Float = 0f,
  /** Mount-calibrated tilt in **degrees**, firmware-parity formulas. */
  val pitchDeg: Float = 0f,
  val rollDeg: Float = 0f,
  /** Angular rate in rad/s, or null when the device has no gyroscope. */
  val gyroX: Float? = null,
  val gyroY: Float? = null,
  val gyroZ: Float? = null,
  val timestamp: Long = 0L,
  val pitchOffsetDeg: Float = 0.0f,
  val rollOffsetDeg: Float = 0.0f,
  val yawRateDps: Float = 0.0f,
  val isAvailable: Boolean = true,
  val timestampNs: Long = 0L,
) {
  val accelX: Float get() = axG
  val accelY: Float get() = ayG
  val accelZ: Float get() = azG
  val forwardAccelG: Float get() = -azG
  val timestampMs: Long get() = timestamp
  val isGyroAvailable: Boolean get() = gyroX != null && gyroY != null && gyroZ != null
}

data class ImuState(
  val availability: Availability = Availability.WAITING,
  val reading: ImuReading? = null,
  val hasAccelerometer: Boolean = false,
  val hasLinearAcceleration: Boolean = false,
  val hasGyroscope: Boolean = false,
  val detail: String = "",
) {
  /** True when ax/ay/az are gravity-free (TYPE_LINEAR_ACCELERATION available). */
  val gravityRemoved: Boolean get() = hasLinearAcceleration
}

data class LocationReading(
  val latitude: Double,
  val longitude: Double,
  /** Horizontal accuracy in metres, null if the provider did not report it. */
  val accuracyM: Float?,
  /** Ground speed in m/s, null when the fix carries no speed. */
  val speedMps: Float?,
  val timestamp: Long,
)

data class LocationState(
  val availability: Availability = Availability.WAITING,
  val reading: LocationReading? = null,
  val detail: String = "",
)

data class BatteryState(
  val availability: Availability = Availability.WAITING,
  val percent: Int? = null,
  val charging: Boolean = false,
  val detail: String = "",
)

data class NetworkState(val online: Boolean = false, val transport: String = "unknown")

enum class CameraRunState {
  STOPPED,
  STARTING,
  ACTIVE,
  ERROR,
}

data class CameraState(
  val availability: Availability = Availability.WAITING,
  val runState: CameraRunState = CameraRunState.STOPPED,
  val visibility: VisibilityResult? = null,
  /** Measured analyser throughput (frames actually scored per second). */
  val analysisFps: Float = 0f,
  /** Measured live video streaming throughput to backend (fps). */
  val streamingFps: Float = 0f,
  val framesAnalysed: Long = 0,
  val framesUploaded: Long = 0,
  /** Frames discarded because the upload queue was full — bounded-queue back-pressure. */
  val framesDropped: Long = 0,
  val lastUploadAt: Long? = null,
  val resolution: String = "—",
  val detail: String = "",
) {
  val isActive: Boolean get() = runState == CameraRunState.ACTIVE
}

enum class BridgeKind {
  /** No ESP32 bridge selected — ultrasonic/IR are reported as NOT AVAILABLE. */
  NONE,

  /** Real ESP32 over USB-OTG serial. */
  USB,

  /** Simulated link for testing without hardware. Never silently enabled. */
  MOCK,
}

data class BridgeState(
  val kind: BridgeKind = BridgeKind.NONE,
  val status: LinkStatus = LinkStatus.Disconnected,
  val telemetry: BridgeTelemetry? = null,
  val lastLineAt: Long? = null,
  val linesParsed: Long = 0,
  val linesRejected: Long = 0,
  val commandsForwarded: Long = 0,
  val deviceName: String? = null,
  val recentLines: List<String> = emptyList(),
) {
  val isConnected: Boolean get() = status is LinkStatus.Connected

  /** True when the values being merged into telemetry are simulated, not measured. */
  val isSimulated: Boolean get() = kind == BridgeKind.MOCK
}

enum class UplinkTransport {
  /** Preferred: `WS /ws/ingest/{vehicle_id}` — carries the command downlink too. */
  WEBSOCKET,

  /** Fallback: `POST /api/vehicles/{id}/telemetry`, commands returned inline. */
  HTTP,
}

sealed interface SocketState {
  data object Idle : SocketState

  data object Connecting : SocketState

  data class Connected(val vehicleId: String, val tickHz: Float?) : SocketState

  data class Reconnecting(val attempt: Int, val maxAttempts: Int, val nextDelayMs: Long) : SocketState

  /** Bounded retry budget spent — requires an explicit operator retry. */
  data class RetryExhausted(val attempts: Int, val lastError: String) : SocketState

  data class Failed(val reason: String) : SocketState
}

data class BackendState(
  val baseUrl: String = "",
  val loggedIn: Boolean = false,
  val username: String? = null,
  val role: String? = null,
  val socket: SocketState = SocketState.Idle,
  val transport: UplinkTransport = UplinkTransport.WEBSOCKET,
  /** Round-trip time of `GET /api/health`, the only honest latency probe available. */
  val latencyMs: Long? = null,
  val lastSentAt: Long? = null,
  val lastAcceptedAt: Long? = null,
  val framesSent: Long = 0,
  val framesFailed: Long = 0,
  /** Measured uplink rate over a short sliding window. */
  val txHz: Float = 0f,
  val lastCommand: CommandFrame? = null,
  val lastCommandAt: Long? = null,
  /**
   * The backend's most recent `{"type":"advisory", …}` frame, or null if none has arrived.
   *
   * This is the control room's own assessment mirrored onto the handset, not a local
   * computation — see [com.minesafety.roboeye.net.AdvisoryFrame]. It arrives only on the
   * WebSocket transport; the HTTP fallback returns commands and nothing else.
   */
  val advisory: AdvisoryFrame? = null,
  /** When [advisory] arrived, by handset clock — the only clock we can age it against. */
  val advisoryAt: Long? = null,
  val lastError: String? = null,
) {
  val isConnected: Boolean
    get() = socket is SocketState.Connected || (transport == UplinkTransport.HTTP && lastAcceptedAt != null)

  /**
   * True when the advisory on screen is older than the backend's own comm timeout.
   *
   * One advisory arrives per accepted frame at ≈2 Hz, so 3 s of silence is many missed frames —
   * and 3 s is precisely when the backend's `COMM_TIMEOUT_S` stops trusting this node's values
   * too. Past that the words on screen are a description of the past, and the UI must say so
   * rather than let an old "NORMAL" imply a current all-clear.
   */
  fun advisoryStale(now: Long): Boolean {
    val at = advisoryAt ?: return false
    return now - at > ADVISORY_STALE_MS
  }

  private companion object {
    const val ADVISORY_STALE_MS = 3_000L
  }
}

// ---------------------------------------------------------------------------
// Aggregate node status (single source of truth for the UI)
// ---------------------------------------------------------------------------

data class NodeStatus(
  val running: Boolean = false,
  val vehicleId: String = "ROVER-01",
  val imu: ImuState = ImuState(),
  val location: LocationState = LocationState(),
  val battery: BatteryState = BatteryState(),
  val network: NetworkState = NetworkState(),
  val health: com.minesafety.roboeye.sensors.HealthState = com.minesafety.roboeye.sensors.HealthState(),
  val camera: CameraState = CameraState(),
  val bridge: BridgeState = BridgeState(),
  val backend: BackendState = BackendState(),
  val mirror: MirrorVerdict = MirrorVerdict.nominal(),
  val startedAt: Long? = null,
  /**
   * Which IMU the **last transmitted frame** actually used, as decided by
   * [TelemetryAssembler]. Shown verbatim on the dashboard so the operator can see whether
   * the tilt on screen came from the chassis or from the handset.
   */
  val imuSource: String = "NOT AVAILABLE",
  /** Whether the bridge was delivering fresh ranges when the last frame was assembled. */
  val bridgeFresh: Boolean = false,
  val radarObjects: List<TrackedObject> = emptyList(),
  val phonePerceptionRunning: Boolean = false,
  val perceptionFps: Float = 0f,
  val perceptionLatencyMs: Long = 0L,
  val modelName: String = "None",
  val selectedModelId: com.minesafety.roboeye.perception.model.DetectorModelId = com.minesafety.roboeye.perception.model.DetectorModelId.YOLO11N,
  val benchmarkSnapshot: com.minesafety.roboeye.perception.benchmark.DetectorPerformanceSnapshot? = null,
  val benchmarkReport: com.minesafety.roboeye.perception.benchmark.DeviceBenchmarkReport? = null,
  val geometricVisionResult: com.minesafety.roboeye.vision.GeometricVisionResult? = null,
  val localPose: com.minesafety.roboeye.localization.LocalPose? = null,
  val localMapState: com.minesafety.roboeye.mapping.LocalMapState? = null,
)
