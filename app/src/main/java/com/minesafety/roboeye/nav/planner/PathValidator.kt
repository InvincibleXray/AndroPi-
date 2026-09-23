package com.minesafety.roboeye.nav.planner

import com.minesafety.roboeye.mapping.MapCellState
import com.minesafety.roboeye.nav.model.NavigationWorldModel
import kotlin.math.hypot

/**
 * Result of validating an active planned path against real-time spatial map evidence.
 */
data class PathValidationResult(
    val isValid: Boolean,
    val reason: String? = null,
    val blockedWaypointIndex: Int? = null,
)

/**
 * Continuous path validator detecting newly emerged obstacles or staleness along the active path.
 */
class PathValidator(
    private val maxPathAgeMs: Long = 5_000L,
) {
    fun validatePath(
        path: PlannedPath,
        worldModel: NavigationWorldModel,
        nowMs: Long = System.currentTimeMillis(),
    ): PathValidationResult {
        if (path.isEmpty) {
            return PathValidationResult(false, "Path is empty")
        }

        if (nowMs - path.generatedAtMs > maxPathAgeMs) {
            return PathValidationResult(false, "Path expired (${nowMs - path.generatedAtMs} ms > $maxPathAgeMs ms)")
        }

        if (!worldModel.isLocalizationValid) {
            return PathValidationResult(false, "Localization lost (${worldModel.pose.trackingState})")
        }

        val map = worldModel.spatialMap
        val inflationCells = (worldModel.inflationRadiusM / map.resolutionM).toInt().coerceAtLeast(1)

        val stepSizeM = map.resolutionM * 0.5f

        // Validate continuous swept volume along all path segments
        if (path.waypoints.size == 1) {
            val wp = path.waypoints[0]
            val gx = ((wp.xM / map.resolutionM) + map.gridWidth / 2f).toInt()
            val gy = ((wp.yM / map.resolutionM) + map.gridHeight / 2f).toInt()
            if (gx !in 0 until map.gridWidth || gy !in 0 until map.gridHeight) {
                return PathValidationResult(false, "Waypoint 0 outside map bounds", 0)
            }
            if (isOccupiedWithFootprint(gx, gy, inflationCells, map)) {
                return PathValidationResult(false, "Obstacle detected near waypoint 0 at ($gx, $gy)", 0)
            }
        } else {
            for (i in 0 until path.waypoints.size - 1) {
                val w1 = path.waypoints[i]
                val w2 = path.waypoints[i + 1]
                val dist = hypot(w2.xM - w1.xM, w2.yM - w1.yM)
                val steps = (dist / stepSizeM).toInt().coerceAtLeast(1)

                for (s in 0..steps) {
                    val t = s.toFloat() / steps
                    val px = w1.xM + t * (w2.xM - w1.xM)
                    val py = w1.yM + t * (w2.yM - w1.yM)

                    val gx = ((px / map.resolutionM) + map.gridWidth / 2f).toInt()
                    val gy = ((py / map.resolutionM) + map.gridHeight / 2f).toInt()

                    if (gx !in 0 until map.gridWidth || gy !in 0 until map.gridHeight) {
                        return PathValidationResult(false, "Path segment $i outside map bounds", i)
                    }

                    if (isOccupiedWithFootprint(gx, gy, inflationCells, map)) {
                        return PathValidationResult(
                            false,
                            "Obstacle detected along path segment $i at ($gx, $gy)",
                            i,
                        )
                    }
                }
            }
        }

        return PathValidationResult(true)
    }

    private fun isOccupiedWithFootprint(
        gx: Int,
        gy: Int,
        inflationCells: Int,
        map: com.minesafety.roboeye.mapping.LocalSpatialMap,
    ): Boolean {
        for (ix in -inflationCells..inflationCells) {
            for (iy in -inflationCells..inflationCells) {
                if (hypot(ix.toDouble(), iy.toDouble()) <= inflationCells) {
                    val nx = gx + ix
                    val ny = gy + iy
                    if (nx in 0 until map.gridWidth && ny in 0 until map.gridHeight) {
                        if (map.cells[nx][ny].state == MapCellState.OCCUPIED) {
                            return true
                        }
                    }
                }
            }
        }
        return false
    }
}
