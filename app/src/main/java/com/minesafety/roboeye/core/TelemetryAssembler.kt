package com.minesafety.roboeye.core

import com.minesafety.roboeye.nav.model.DistanceCertainty
import com.minesafety.roboeye.nav.model.WorldObject
import com.minesafety.roboeye.net.SemanticObjectTelemetryDto
import com.minesafety.roboeye.net.TelemetryFrame

/**
 * Assembles pure Phone Telemetry into a [TelemetryFrame].
 *
 * Physical rover distance ranging (ultrasonic) and line sensors (IR) belong exclusively
 * to the ESP32 and are omitted here. The Android phone owns and reports only what its
 * onboard sensors actually measure:
 *
 * | Field | Owner | Provenance / Rule |
 * |---|---|---|
 * | `visibility`, `camera_available` | Phone Camera | Local contrast/sharpness score and camera run state |
 * | `imu_pitch`, `imu_roll` | Phone IMU | Mount-calibrated orientation in degrees |
 * | `imu_ax`, `imu_ay`, `imu_az` | Phone IMU | Linear acceleration in g (peak-hold or instantaneous) |
 * | `latitude`, `longitude`, `speed` | Phone GPS | FusedLocationProvider; null when no fix |
 * | `battery` | Phone Battery | Handset charge percentage (0..100) |
 * | `ultrasonic_*`, `ir_*` | ESP32 | Omitted (null) — physical sensors belong to ESP32 |
 * | `semantic_objects` | Phone World Model | Observational tracked semantic objects from NavigationWorldModel |
 */
object TelemetryAssembler {

  data class Assembled(
    val frame: TelemetryFrame,
    val minObstacleM: Float? = null,
    val accelMagG: Float?,
    val imuSource: String,
    val bridgeFresh: Boolean = false,
  )

  fun build(
    vehicleId: String,
    now: Long,
    imu: ImuState,
    location: LocationState,
    battery: BatteryState,
    camera: CameraState,
    bridge: BridgeState = BridgeState(),
    peakAccel: Triple<Float, Float, Float>? = null,
    semanticObjects: List<WorldObject> = emptyList(),
  ): Assembled {
    val phoneImu = imu.reading.takeIf { imu.availability.isUsable }

    val pitch = phoneImu?.pitchDeg
    val roll = phoneImu?.rollDeg

    val ax: Float?
    val ay: Float?
    val az: Float?
    if (peakAccel != null) {
      ax = peakAccel.first
      ay = peakAccel.second
      az = peakAccel.third
    } else {
      ax = phoneImu?.axG
      ay = phoneImu?.ayG
      az = phoneImu?.azG
    }

    val visibility = camera.visibility?.score.takeIf { camera.isActive }
    val fix = location.reading.takeIf { location.availability.isUsable }

    val sanitizedSemantic = semanticObjects.map { obj ->
      val dist = if (obj.distanceCertainty == DistanceCertainty.UNKNOWN) {
        null
      } else {
        obj.roverDistanceM.sanitizeDistance()
      }
      SemanticObjectTelemetryDto(
        trackId = obj.trackId,
        label = obj.label,
        objectType = obj.objectType.name,
        confidence = if (obj.confidence.isFinite()) obj.confidence else 0.0f,
        timestampMs = obj.timestampMs,
        roverDistanceM = dist,
        roverBearingDeg = obj.roverBearingDeg.sanitize(),
        roverXM = obj.roverXM.sanitize(),
        roverYM = obj.roverYM.sanitize(),
        mapXM = obj.mapXM.sanitize(),
        mapYM = obj.mapYM.sanitize(),
        widthM = obj.widthM.sanitizeDistance(),
        heightM = obj.heightM.sanitizeDistance(),
        relativeMotion = obj.relativeMotion.name,
        confirmed = obj.isConfirmed,
        coasting = obj.isCoasting,
        threatScore = obj.threatScore.sanitize()?.coerceIn(0.0f, 1.0f),
        distanceCertainty = obj.distanceCertainty.name,
      )
    }

    val frame =
      TelemetryFrame(
        vehicleId = vehicleId,
        timestamp = Units.isoUtc(now),
        // Omitted when there is no GPS fix: avoid drifting inertial speed estimates
        speed = fix?.speedMps,
        battery = battery.percent?.toFloat(),
        visibility = visibility,
        ultrasonicFront = null,
        ultrasonicLeft = null,
        ultrasonicRight = null,
        irLeft = null,
        irCenter = null,
        irRight = null,
        imuAx = ax,
        imuAy = ay,
        imuAz = az,
        imuPitch = pitch,
        imuRoll = roll,
        latitude = fix?.latitude,
        longitude = fix?.longitude,
        cameraAvailable = camera.isActive,
        semanticObjects = sanitizedSemantic,
      )

    val accelMag = if (ax != null && ay != null) Units.accelMagnitudeG(ax, ay) else null

    return Assembled(
      frame = frame,
      minObstacleM = null,
      accelMagG = accelMag,
      imuSource = if (phoneImu != null) "Phone IMU" else "NOT AVAILABLE",
      bridgeFresh = false,
    )
  }

  private fun Float?.sanitize(): Float? =
    if (this != null && this.isFinite()) this else null

  private fun Float?.sanitizeDistance(): Float? =
    if (this != null && this.isFinite() && this > 0f) this else null
}
