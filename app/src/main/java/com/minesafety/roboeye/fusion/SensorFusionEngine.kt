package com.minesafety.roboeye.fusion

import com.minesafety.roboeye.core.model.Esp32SensorState
import com.minesafety.roboeye.core.model.ObstacleObservation
import com.minesafety.roboeye.core.model.PhoneSensorState
import com.minesafety.roboeye.vision.VisionOutput
import kotlin.math.min

/**
 * Fused world model state resulting from combining phone IMU/GNSS, ESP32 ultrasonics/ToF, and vision.
 */
data class FusedWorldState(
    val fusedObstacles: List<ObstacleObservation> = emptyList(),
    val minObstacleDistanceM: Float? = null,
    val vehiclePitchDeg: Float = 0.0f,
    val vehicleRollDeg: Float = 0.0f,
    val isClearAhead: Boolean = true,
    val freeSpace: FusedFreeSpace? = null,
    val crossCheckLeft: SectorCrossCheckResult? = null,
    val crossCheckCenter: SectorCrossCheckResult? = null,
    val crossCheckRight: SectorCrossCheckResult? = null,
    val sensorHealth: Map<String, ReadingStatus> = emptyMap(),
    val isRotationalMotion: Boolean = false,
    val overallConfidence: Float = 1.0f,
    val timestampMs: Long = System.currentTimeMillis(),
)

/**
 * Multi-sensor fusion engine boundary interface.
 */
interface SensorFusionEngine {
    fun fuse(
        phoneState: PhoneSensorState,
        esp32State: Esp32SensorState,
        visionOutput: VisionOutput?,
    ): FusedWorldState
}

/**
 * Deterministic multi-sensor fusion engine combining physical distance sensors,
 * phone IMU kinematics, and lightweight geometric vision optical flow.
 */
class DefaultSensorFusionEngine(
    val validator: SensorValidator = SensorValidator(),
    val motionCompensator: VisionMotionCompensator = VisionMotionCompensator(),
    val crossChecker: RangeVisionCrossCheck = RangeVisionCrossCheck(),
    val freeSpaceEstimator: GeometricFreeSpaceEstimator = GeometricFreeSpaceEstimator(),
) : SensorFusionEngine {

    // Temporal persistence counters for sector obstacles (0..5 scale)
    private var leftPersistence = 0
    private var centerPersistence = 0
    private var rightPersistence = 0

    @Synchronized
    override fun fuse(
        phoneState: PhoneSensorState,
        esp32State: Esp32SensorState,
        visionOutput: VisionOutput?,
    ): FusedWorldState {
        val nowMs = System.currentTimeMillis()

        // 1. Validate physical distance readings from ESP32
        val vFront = validator.validateRange(
            "FRONT",
            esp32State.ultrasonicFrontM,
            esp32State.timestampMs,
            nowMs,
            DistanceSensorMount.DEFAULT_FRONT
        )
        val vLeft = validator.validateRange(
            "LEFT",
            esp32State.ultrasonicLeftM,
            esp32State.timestampMs,
            nowMs,
            DistanceSensorMount.DEFAULT_LEFT
        )
        val vRight = validator.validateRange(
            "RIGHT",
            esp32State.ultrasonicRightM,
            esp32State.timestampMs,
            nowMs,
            DistanceSensorMount.DEFAULT_RIGHT
        )
        val vTof = validator.validateRange(
            "TOF",
            esp32State.tofFrontM,
            esp32State.timestampMs,
            nowMs,
            DistanceSensorMount.DEFAULT_TOF
        )

        val health = mapOf(
            "ULTRASONIC_FRONT" to vFront.status,
            "ULTRASONIC_LEFT" to vLeft.status,
            "ULTRASONIC_RIGHT" to vRight.status,
            "TOF_FRONT" to vTof.status,
            "PHONE_IMU" to if (validator.isImuFresh(phoneState.timestampMs, nowMs)) ReadingStatus.VALID else ReadingStatus.STALE,
            "VISION" to if (visionOutput != null && validator.isVisionFresh(visionOutput.timestampNs / 1_000_000L, nowMs)) ReadingStatus.VALID else ReadingStatus.UNAVAILABLE,
        )

        // Choose most critical / closest valid front range between Ultrasonic and ToF
        val effectiveCenterRange = when {
            vFront.isValid && vTof.isValid -> {
                val d = min(vFront.distanceM!!, vTof.distanceM!!)
                vFront.copy(distanceM = d)
            }
            vTof.isValid -> vTof
            else -> vFront
        }

        // 2. Evaluate phone IMU rotational motion compensation
        val imu = phoneState.imu
        val comp = motionCompensator.evaluate(imu)
        val isRotating = comp.isRotationalMotion
        val visionDamping = comp.confidenceMultiplier

        // 3. Extract visual observations by sector
        var leftVisualTtc: Float? = null
        var leftVisualConf = 0.0f
        var centerVisualTtc: Float? = null
        var centerVisualConf = 0.0f
        var rightVisualTtc: Float? = null
        var rightVisualConf = 0.0f

        if (visionOutput != null && !isRotating) {
            for (obs in visionOutput.obstacles) {
                val ttc = obs.timeToCollisionSec
                val conf = obs.confidence * visionDamping
                when {
                    obs.bearingDeg < -15.0f -> {
                        leftVisualTtc = ttc
                        leftVisualConf = conf
                    }
                    obs.bearingDeg > 15.0f -> {
                        rightVisualTtc = ttc
                        rightVisualConf = conf
                    }
                    else -> {
                        centerVisualTtc = ttc
                        centerVisualConf = conf
                    }
                }
            }
        }

        // 4. Sector Cross-Checks
        val crossLeft = crossChecker.crossCheckSector(CorridorSector.LEFT, vLeft, leftVisualTtc, leftVisualConf)
        val crossCenter = crossChecker.crossCheckSector(CorridorSector.CENTER, effectiveCenterRange, centerVisualTtc, centerVisualConf)
        val crossRight = crossChecker.crossCheckSector(CorridorSector.RIGHT, vRight, rightVisualTtc, rightVisualConf)

        // 5. Update temporal persistence
        updatePersistence(crossLeft.agreement, CorridorSector.LEFT)
        updatePersistence(crossCenter.agreement, CorridorSector.CENTER)
        updatePersistence(crossRight.agreement, CorridorSector.RIGHT)

        // 6. Geometric Free-Space Estimation
        val freeSpace = freeSpaceEstimator.estimate(crossLeft, crossCenter, crossRight)

        // 7. Compile fused obstacle observations
        val obstacles = ArrayList<ObstacleObservation>()

        fun addObstacleIfConfirmed(sector: CorridorSector, cross: SectorCrossCheckResult, persistence: Int) {
            val isHazard = cross.agreement == CrossCheckAgreement.CONSISTENT_OBSTACLE ||
                    (cross.agreement == CrossCheckAgreement.RANGE_ONLY && (cross.physicalRangeM ?: 5f) <= 1.5f) ||
                    (cross.agreement == CrossCheckAgreement.VISION_ONLY && (cross.visualTtcSec ?: 10f) <= 2.5f) ||
                    cross.agreement == CrossCheckAgreement.CONFLICTING

            if (isHazard && persistence >= 1) {
                val (bearing, mount) = when (sector) {
                    CorridorSector.LEFT -> Pair(45.0f, DistanceSensorMount.DEFAULT_LEFT)
                    CorridorSector.CENTER -> Pair(0.0f, DistanceSensorMount.DEFAULT_FRONT)
                    CorridorSector.RIGHT -> Pair(-45.0f, DistanceSensorMount.DEFAULT_RIGHT)
                }

                val dist = cross.physicalRangeM ?: ((cross.visualTtcSec ?: 2.0f) * 0.5f).coerceIn(0.2f, 4.0f)
                val source = when (cross.agreement) {
                    CrossCheckAgreement.CONSISTENT_OBSTACLE -> "FUSED_DUAL_RANGE_VISION"
                    CrossCheckAgreement.RANGE_ONLY -> "ESP32_RANGE"
                    CrossCheckAgreement.VISION_ONLY -> "OPTICAL_FLOW_TTC"
                    CrossCheckAgreement.CONFLICTING -> "CONFLICTING_SENSOR_ALERT"
                    CrossCheckAgreement.CONSISTENT_CLEAR, CrossCheckAgreement.UNKNOWN -> "SENSOR_FUSION"
                }

                obstacles.add(
                    ObstacleObservation(
                        id = "FUSED_${sector.name}_${nowMs}",
                        distanceM = dist,
                        bearingDeg = bearing,
                        timeToCollisionSec = cross.visualTtcSec,
                        source = source,
                        confidence = cross.confidence,
                        timestampMs = nowMs,
                    )
                )
            }
        }

        addObstacleIfConfirmed(CorridorSector.LEFT, crossLeft, leftPersistence)
        addObstacleIfConfirmed(CorridorSector.CENTER, crossCenter, centerPersistence)
        addObstacleIfConfirmed(CorridorSector.RIGHT, crossRight, rightPersistence)

        val minObsDist = obstacles.map { it.distanceM }.minOrNull()

        // Vehicle pitch/roll prioritizing phone IMU, fallback to chassis IMU
        val pitch = phoneState.imu?.pitchDeg ?: (esp32State.chassisImuPitchDeg ?: 0.0f)
        val roll = phoneState.imu?.rollDeg ?: (esp32State.chassisImuRollDeg ?: 0.0f)

        return FusedWorldState(
            fusedObstacles = obstacles,
            minObstacleDistanceM = minObsDist,
            vehiclePitchDeg = pitch,
            vehicleRollDeg = roll,
            isClearAhead = freeSpace.isClearAhead,
            freeSpace = freeSpace,
            crossCheckLeft = crossLeft,
            crossCheckCenter = crossCenter,
            crossCheckRight = crossRight,
            sensorHealth = health,
            isRotationalMotion = isRotating,
            overallConfidence = freeSpace.overallConfidence,
            timestampMs = nowMs,
        )
    }

    private fun updatePersistence(agreement: CrossCheckAgreement, sector: CorridorSector) {
        val isObstacle = agreement == CrossCheckAgreement.CONSISTENT_OBSTACLE ||
                agreement == CrossCheckAgreement.RANGE_ONLY ||
                agreement == CrossCheckAgreement.VISION_ONLY ||
                agreement == CrossCheckAgreement.CONFLICTING

        when (sector) {
            CorridorSector.LEFT -> {
                leftPersistence = if (isObstacle) min(5, leftPersistence + 1) else maxOf(0, leftPersistence - 1)
            }
            CorridorSector.CENTER -> {
                centerPersistence = if (isObstacle) min(5, centerPersistence + 1) else maxOf(0, centerPersistence - 1)
            }
            CorridorSector.RIGHT -> {
                rightPersistence = if (isObstacle) min(5, rightPersistence + 1) else maxOf(0, rightPersistence - 1)
            }
        }
    }

    fun reset() {
        leftPersistence = 0
        centerPersistence = 0
        rightPersistence = 0
        validator.reset()
    }
}
