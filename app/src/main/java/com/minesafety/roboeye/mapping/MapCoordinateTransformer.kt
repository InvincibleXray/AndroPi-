package com.minesafety.roboeye.mapping

import com.minesafety.roboeye.fusion.CorridorSector
import com.minesafety.roboeye.fusion.Point2D
import com.minesafety.roboeye.localization.LocalPose
import com.minesafety.roboeye.vision.OccupancyCellState
import kotlin.math.cos
import kotlin.math.roundToInt
import kotlin.math.sin

/**
 * 2D rigid-body coordinate transformer that converts egocentric polar sector observations
 * and body-frame positions into persistent local map coordinates.
 */
class MapCoordinateTransformer(
    val gridResolutionM: Float = 0.25f,
    val gridWidthCells: Int = 24,
    val gridHeightCells: Int = 24,
) {
    val originGridX: Int = gridWidthCells / 2
    val originGridY: Int = gridHeightCells / 2

    // Representative center radial distances for the 4 Phase 4 rings
    private val ringDistancesM = floatArrayOf(0.45f, 1.10f, 2.00f, 3.25f)

    // Sector azimuth angles relative to rover forward (+X forward, +Y left, CCW positive)
    private val sectorAnglesDeg = mapOf(
        CorridorSector.LEFT to 22.5f,
        CorridorSector.CENTER to 0.0f,
        CorridorSector.RIGHT to -22.5f,
    )

    /**
     * Transforms a polar sector and range ring into body-frame coordinates.
     */
    fun polarToBody(sector: CorridorSector, ringIndex: Int): Point2D {
        val r = ringDistancesM[ringIndex.coerceIn(0, ringDistancesM.size - 1)]
        val azimuthDeg = sectorAnglesDeg[sector] ?: 0.0f
        val azRad = Math.toRadians(azimuthDeg.toDouble())
        val xb = (r * cos(azRad)).toFloat()
        val yb = (r * sin(azRad)).toFloat()
        return Point2D(xb, yb)
    }

    /**
     * Transforms body-frame coordinates into local map-frame coordinates using rover pose.
     */
    fun bodyToMap(bodyPoint: Point2D, roverPose: LocalPose): Point2D {
        val yawRad = Math.toRadians(roverPose.yawDeg.toDouble())
        val cosYaw = cos(yawRad).toFloat()
        val sinYaw = sin(yawRad).toFloat()

        val xm = roverPose.xM + bodyPoint.xM * cosYaw - bodyPoint.yM * sinYaw
        val ym = roverPose.yM + bodyPoint.xM * sinYaw + bodyPoint.yM * cosYaw
        return Point2D(xm, ym)
    }

    /**
     * Converts map coordinates (meters) to integer grid indices (gridX, gridY).
     * Returns null if outside grid bounds.
     */
    fun mapToGrid(mapPoint: Point2D): Pair<Int, Int>? {
        val gx = originGridX + (mapPoint.xM / gridResolutionM).roundToInt()
        val gy = originGridY + (mapPoint.yM / gridResolutionM).roundToInt()
        return if (gx in 0 until gridWidthCells && gy in 0 until gridHeightCells) {
            Pair(gx, gy)
        } else {
            null
        }
    }

    /**
     * Maps Phase 4 OccupancyCellState to Phase 5 MapCellState.
     */
    fun mapCellState(state: OccupancyCellState): MapCellState = when (state) {
        OccupancyCellState.FREE -> MapCellState.FREE
        OccupancyCellState.OCCUPIED -> MapCellState.OCCUPIED
        OccupancyCellState.UNCERTAIN -> MapCellState.UNCERTAIN
    }
}
