package com.minesafety.roboeye.net

import com.minesafety.roboeye.core.RadarConfig
import com.minesafety.roboeye.core.RoadGeometryState
import com.minesafety.roboeye.core.TrackedObject
import com.minesafety.roboeye.core.Units
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale
import java.util.TimeZone

object RadarPacketBuilder {

    private val isoFormat = SimpleDateFormat("yyyy-MM-dd'T'HH:mm:ss.SSS'Z'", Locale.US).apply {
        timeZone = TimeZone.getTimeZone("UTC")
    }

    /**
     * Builds a VisionRadarPacketDto conforming to backend perception schemas.
     */
    fun buildPacket(
        config: RadarConfig,
        trackedObjects: List<TrackedObject>,
        visibilityScore: Float,
        roadGeometry: RoadGeometryState? = null,
        nowMs: Long = System.currentTimeMillis(),
    ): VisionRadarPacketDto {
        val isoTimestamp = synchronized(isoFormat) {
            isoFormat.format(Date(nowMs))
        }

        val wireObjects = trackedObjects.map { obj ->
            VisionRadarObjectDto(
                type = obj.type.wireValue,
                label = obj.label,
                confidence = Units.clamp01(obj.confidence),
                estimatedDistanceM = Units.round1(obj.estimatedDistanceM),
                relativeMotion = obj.relativeMotion.wireValue,
                directionDeg = Units.round1(obj.bearingDeg),
                distanceIsMeasured = false,
                objectId = obj.id,
                isCoasting = obj.isCoasting,
                observationAgeS = Units.round2(obj.stalenessMs / 1000f),
                metricWidthM = if (obj.metricWidthM > 0.0f) obj.metricWidthM else null,
                metricHeightM = if (obj.metricHeightM > 0.0f) obj.metricHeightM else null,
                isOnRoad = if (roadGeometry?.isSupported == true) obj.isOnRoad else null,
            )
        }.take(24)

        val avgTargetConf = if (wireObjects.isNotEmpty()) {
            wireObjects.map { it.confidence }.average().toFloat()
        } else {
            visibilityScore
        }
        val perceptionConf = Units.clamp01(avgTargetConf * 0.7f + visibilityScore * 0.3f)

        val road = roadGeometry?.takeIf { it.isSupported }

        return VisionRadarPacketDto(
            vehicleId = config.vehicleId,
            nodeId = config.nodeId,
            nodeType = "VISION_RADAR",
            timestamp = isoTimestamp,
            scanAngle = 0.0f,
            scanDirection = "FRONT",
            scanPriority = "HIGH",
            scanRateDps = null,
            visibility = Units.clamp01(visibilityScore),
            perceptionConfidence = Units.round2(perceptionConf),
            objects = wireObjects,
            simulated = false,
            roadStatus = road?.roadStatus?.wireValue,
            roadBlockagePct = road?.roadBlockagePct,
            traversableWidthM = road?.maxTraversableCorridorM,
            isPassable = road?.isPassable,
        )
    }
}
