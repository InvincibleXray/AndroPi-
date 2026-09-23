package com.minesafety.roboeye.core

/**
 * Normalized 2D bounding box where coordinates are in [0.0, 1.0].
 * Pure Kotlin data structure that executes deterministically on both Android and JVM unit tests.
 */
data class NormalizedRect(
    val left: Float = 0.0f,
    val top: Float = 0.0f,
    val right: Float = 0.0f,
    val bottom: Float = 0.0f,
) {
    val width: Float get() = (right - left).coerceAtLeast(0.0f)
    val height: Float get() = (bottom - top).coerceAtLeast(0.0f)
    val centerX: Float get() = left + width * 0.5f
    val centerY: Float get() = top + height * 0.5f
}

/**
 * High-level object classification matching backend `PerceptionObjectType`.
 */
enum class PerceptionObjectType(val wireValue: String) {
    VEHICLE("VEHICLE"),
    PERSON("PERSON"),
    OBSTACLE("OBSTACLE"),
    ROAD_OBSTRUCTION("ROAD_OBSTRUCTION"),
    LARGE_OBJECT("LARGE_OBJECT"),
}

/**
 * Relative motion classification matching backend `RelativeMotion`.
 */
enum class RelativeMotion(val wireValue: String) {
    APPROACHING("APPROACHING"),
    RECEDING("RECEDING"),
    STATIONARY("STATIONARY"),
    UNKNOWN("UNKNOWN"),
}

/**
 * Scan direction matching backend `ScanDirection`.
 */
enum class ScanDirection(val wireValue: String) {
    FRONT("FRONT"),
    FRONT_LEFT("FRONT_LEFT"),
    FRONT_RIGHT("FRONT_RIGHT"),
    LEFT("LEFT"),
    RIGHT("RIGHT"),
    REAR("REAR"),
    REAR_LEFT("REAR_LEFT"),
    REAR_RIGHT("REAR_RIGHT"),
}

/**
 * Scan priority matching backend `ScanPriority`.
 */
enum class ScanPriority(val wireValue: String) {
    HIGH("HIGH"),
    MEDIUM("MEDIUM"),
    LOW("LOW"),
}

/**
 * Backend connection status for UI and diagnostic telemetry.
 */
enum class BackendConnectionStatus(val label: String) {
    CONNECTED("CONNECTED"),
    CONNECTING("CONNECTING..."),
    OFFLINE("OFFLINE"),
    UNAUTHORIZED("AUTH FAILED"),
    ERROR("COMM ERROR"),
}

/**
 * Road traversability status.
 *
 * [wireValue] is `null` for [UNKNOWN] on purpose: the backend's `RoadStatus` has no member for
 * "not perceived", so the only truthful way to report that state over the wire is to omit the
 * field. Making the absence part of the enum means a caller cannot accidentally serialise a
 * status the control room would read as a real assessment.
 */
enum class RoadStatus(val label: String, val wireValue: String?) {
    CLEAR("ROAD CLEAR", "CLEAR"),
    PARTIALLY_BLOCKED("PARTIALLY BLOCKED", "PARTIALLY_BLOCKED"),
    BLOCKED("ROAD BLOCKED", "BLOCKED"),
    OFF_ROAD("OFF ROAD", "OFF_ROAD"),
    UNKNOWN("ROAD PERCEPTION UNAVAILABLE", null),
}

/**
 * Geometric road boundaries and traversability state.
 *
 * **This app has no road-perception producer.** Nothing in the pipeline segments a drivable
 * area, extracts road edges, or detects a horizon from pixels, so [isSupported] is `false` in
 * every configuration that ships today and every other field is meaningless. See
 * [com.minesafety.roboeye.perception.RoadSegmentationEngine] for the seam a real producer
 * would fill.
 *
 * Every read site must branch on [isSupported] first. The numeric defaults are deliberately
 * zero and [isPassable] deliberately `false` so that a read site which forgets to branch
 * degrades to "no clearance, do not pass" rather than to the previous defaults, which claimed
 * a clear 8.5 m road and would have been read as a measurement.
 */
data class RoadGeometryState(
    val isSupported: Boolean = false,
    val drivableWidthAtRoverM: Float = 0.0f,
    val drivableWidthAtObstacleM: Float = 0.0f,
    val roadBlockagePct: Float = 0.0f,
    val traversableCorridorLeftM: Float = 0.0f,
    val traversableCorridorRightM: Float = 0.0f,
    val maxTraversableCorridorM: Float = 0.0f,
    val isPassable: Boolean = false,
    val roadStatus: RoadStatus = RoadStatus.UNKNOWN,
    val leftBoundaryBearingDeg: Float = 0.0f,
    val rightBoundaryBearingDeg: Float = 0.0f,
    val roadCenterlineOffsetDeg: Float = 0.0f,
    val primaryObstacleId: String? = null,
    val timestampMs: Long = System.currentTimeMillis(),
)

/**
 * Raw detection output from a single camera frame before tracking.
 */
data class RawDetectedObject(
    val boundingBox: NormalizedRect,
    val label: String,
    val type: PerceptionObjectType,
    val confidence: Float,
    val timestampMs: Long = System.currentTimeMillis(),
)

/**
 * A tracked object maintained across consecutive camera frames.
 */
data class TrackedObject(
    val id: String,
    val type: PerceptionObjectType,
    val label: String,
    val boundingBox: NormalizedRect,
    val bearingDeg: Float,
    val estimatedDistanceM: Float,
    val confidence: Float,
    val relativeMotion: RelativeMotion = RelativeMotion.UNKNOWN,
    val firstSeenMs: Long = System.currentTimeMillis(),
    val lastSeenMs: Long = System.currentTimeMillis(),
    val hitCount: Int = 1,
    val isConfirmed: Boolean = true,
    val isPrimary: Boolean = false,
    val threatScore: Float = 0.0f,
    val isCoasting: Boolean = false,
    val metricWidthM: Float = 0.0f,
    val metricHeightM: Float = 0.0f,
    val groundFootprintLeftM: Float = 0.0f,
    val groundFootprintRightM: Float = 0.0f,
    val estimatedMassKg: Float = 0.0f,
    val isOnRoad: Boolean = true,
    val blockageContributionPct: Float = 0.0f,
) {
    val ageMs: Long get() = System.currentTimeMillis() - firstSeenMs
    val stalenessMs: Long get() = System.currentTimeMillis() - lastSeenMs
}


/**
 * Overall runtime diagnostic state of the Robo Radar pipeline.
 *
 * [visibilityScore] is nullable because visibility is *measured from a camera frame*. Before
 * the first frame is scored there is no measurement, and the previous default of `1.0f` stated
 * the strongest possible claim — perfect visibility — at exactly the moment the app knew
 * least. `null` means "not measured yet"; every read site must render that as unknown rather
 * than substituting a number.
 */
data class RadarSystemState(
    val cameraActive: Boolean = false,
    val cameraFps: Float = 0.0f,
    val imuReading: ImuReading = ImuReading(),
    val backendStatus: BackendConnectionStatus = BackendConnectionStatus.OFFLINE,
    val backendLatencyMs: Long = -1L,
    val backendPacketsSent: Long = 0L,
    val lastBackendError: String? = null,
    val visibilityScore: Float? = null,
    val inferenceLatencyMs: Long = 0L,
    val roadGeometry: RoadGeometryState = RoadGeometryState(),
)
