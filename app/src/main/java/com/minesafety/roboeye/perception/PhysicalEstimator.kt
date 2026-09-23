package com.minesafety.roboeye.perception

import com.minesafety.roboeye.core.NormalizedRect
import com.minesafety.roboeye.core.PerceptionObjectType
import com.minesafety.roboeye.core.Units
import kotlin.math.max
import kotlin.math.min
import kotlin.math.tan

/**
 * Metric Physical Dimensions, Ground Footprint, and Mass Estimator.
 */
object PhysicalEstimator {

    const val DENSITY_ROCK_BOULDER_KG_M3 = 2800.0f
    const val DENSITY_VEHICLE_BULK_KG_M3 = 220.0f
    const val DENSITY_PERSON_KG_M3 = 1000.0f
    const val DENSITY_OBSTRUCTION_KG_M3 = 1200.0f
    const val DENSITY_LARGE_EQUIP_KG_M3 = 300.0f

    data class MetricDimensions(
        val widthM: Float,
        val heightM: Float,
        val lateralCenterX: Float,
        val footprintLeftX: Float,
        val footprintRightX: Float,
        val estimatedVolumeM3: Float,
        val estimatedMassKg: Float,
    )

    fun estimateMetrics(
        boundingBox: NormalizedRect,
        type: PerceptionObjectType,
        distanceM: Float,
        cameraHfovDeg: Float = 68.0f,
        cameraVfovDeg: Float = 52.0f,
    ): MetricDimensions {
        val d = distanceM.coerceAtLeast(0.5f)
        val halfHfovRad = (cameraHfovDeg * 0.5f) * Units.DEG_TO_RAD
        val halfVfovRad = (cameraVfovDeg * 0.5f) * Units.DEG_TO_RAD

        val visibleWidthAtD = (2.0f * d * tan(halfHfovRad.toDouble())).toFloat()
        val visibleHeightAtD = (2.0f * d * tan(halfVfovRad.toDouble())).toFloat()

        val widthM = Units.round2(max(0.1f, boundingBox.width * visibleWidthAtD))
        val heightM = Units.round2(max(0.1f, boundingBox.height * visibleHeightAtD))

        val lateralCenterM = Units.round2((boundingBox.centerX - 0.5f) * visibleWidthAtD)
        val footprintLeftM = Units.round2(lateralCenterM - (widthM * 0.5f))
        val footprintRightM = Units.round2(lateralCenterM + (widthM * 0.5f))

        val (shapeFactor, depthRatio, densityKgM3) = when (type) {
            PerceptionObjectType.PERSON -> Triple(0.40f, 0.40f, DENSITY_PERSON_KG_M3)
            PerceptionObjectType.VEHICLE -> Triple(0.65f, 1.80f, DENSITY_VEHICLE_BULK_KG_M3)
            PerceptionObjectType.LARGE_OBJECT -> Triple(0.60f, 1.50f, DENSITY_LARGE_EQUIP_KG_M3)
            PerceptionObjectType.ROAD_OBSTRUCTION -> Triple(0.50f, 0.80f, DENSITY_OBSTRUCTION_KG_M3)
            PerceptionObjectType.OBSTACLE -> Triple(0.52f, 0.90f, DENSITY_ROCK_BOULDER_KG_M3)
        }

        val depthM = max(0.1f, min(widthM, heightM) * depthRatio)
        val volumeM3 = Units.round2(widthM * heightM * depthM * shapeFactor)
        val massKg = Units.round1(volumeM3 * densityKgM3)

        return MetricDimensions(
            widthM = widthM,
            heightM = heightM,
            lateralCenterX = lateralCenterM,
            footprintLeftX = footprintLeftM,
            footprintRightX = footprintRightM,
            estimatedVolumeM3 = volumeM3,
            estimatedMassKg = massKg,
        )
    }

    fun estimateMetricWidth(boundingBox: NormalizedRect, distanceM: Float, cameraHfovDeg: Float = 68.0f): Float {
        return estimateMetrics(boundingBox, PerceptionObjectType.OBSTACLE, distanceM, cameraHfovDeg).widthM
    }

    fun estimateFootprint(
        box: NormalizedRect,
        distanceM: Float,
        bearingDeg: Float = 0.0f,
        cameraHfovDeg: Float = 68.0f,
    ): Pair<Float, Float> {
        val m = estimateMetrics(box, PerceptionObjectType.OBSTACLE, distanceM, cameraHfovDeg)
        return m.footprintLeftX to m.footprintRightX
    }

    fun estimateMass(type: PerceptionObjectType, widthM: Float, heightM: Float): Float {
        val (shapeFactor, depthRatio, densityKgM3) = when (type) {
            PerceptionObjectType.PERSON -> Triple(0.40f, 0.40f, DENSITY_PERSON_KG_M3)
            PerceptionObjectType.VEHICLE -> Triple(0.65f, 1.80f, DENSITY_VEHICLE_BULK_KG_M3)
            PerceptionObjectType.LARGE_OBJECT -> Triple(0.60f, 1.50f, DENSITY_LARGE_EQUIP_KG_M3)
            PerceptionObjectType.ROAD_OBSTRUCTION -> Triple(0.50f, 0.80f, DENSITY_OBSTRUCTION_KG_M3)
            PerceptionObjectType.OBSTACLE -> Triple(0.52f, 0.90f, DENSITY_ROCK_BOULDER_KG_M3)
        }
        val depthM = max(0.1f, min(widthM, heightM) * depthRatio)
        val volumeM3 = Units.round2(widthM * heightM * depthM * shapeFactor)
        return Units.round1(volumeM3 * densityKgM3)
    }
}
