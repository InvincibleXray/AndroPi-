package com.minesafety.roboeye.vision

import com.minesafety.roboeye.fusion.CorridorSector
import com.minesafety.roboeye.fusion.VisualFreeSpaceEvidence
import kotlin.math.abs

/**
 * Geometric state of a local visual occupancy cell.
 */
enum class OccupancyCellState {
    FREE,
    OCCUPIED,
    UNCERTAIN
}

/**
 * Single cell in the egocentric local visual occupancy map.
 */
data class OccupancyCell(
    val sector: CorridorSector,
    val ringIndex: Int,
    val minDistanceM: Float,
    val maxDistanceM: Float,
    var state: OccupancyCellState = OccupancyCellState.UNCERTAIN,
    var occupancyProbability: Float = 0.5f,
    var confidence: Float = 0.0f,
)

/**
 * Egocentric 2D Local Visual Occupancy Grid ahead of the rover.
 *
 * Maintains a 12-cell (3 sectors x 4 range rings) occupancy representation
 * derived entirely from camera optical flow, regional TTC, and IMU kinematics.
 *
 * Characteristics:
 * - 4 Range Rings: [0.2m..0.7m], [0.7m..1.5m], [1.5m..2.5m], [2.5m..4.0m]
 * - 3 Corridors: LEFT, CENTER, RIGHT
 * - Short-term temporal memory (~2s) with exponential decay
 * - Accelerated decay during high-yaw rotational maneuvers
 * - Bounded memory, zero dynamic heap allocations in hot loops
 */
class LocalVisualOccupancyGrid(
    val decayFactorPerFrame: Float = 0.85f,
    val turnDecayFactor: Float = 0.60f,
    val minConfidenceThreshold: Float = 0.20f,
) {
    companion object {
        val RING_BOUNDS = listOf(
            0.2f to 0.7f,  // Ring 0: Near / E-stop
            0.7f to 1.5f,  // Ring 1: Caution / Braking
            1.5f to 2.5f,  // Ring 2: Mid-range
            2.5f to 4.0f,  // Ring 3: Far horizon
        )
    }

    // 3 sectors x 4 rings = 12 cells
    val cells: List<OccupancyCell> = CorridorSector.values().flatMap { sector ->
        RING_BOUNDS.mapIndexed { ringIdx, (minD, maxD) ->
            OccupancyCell(
                sector = sector,
                ringIndex = ringIdx,
                minDistanceM = minD,
                maxDistanceM = maxD,
            )
        }
    }

    @Synchronized
    fun reset() {
        for (c in cells) {
            c.state = OccupancyCellState.UNCERTAIN
            c.occupancyProbability = 0.5f
            c.confidence = 0.0f
        }
    }

    @Synchronized
    fun update(
        evidenceList: List<VisualFreeSpaceEvidence>,
        motionState: MotionState,
        yawRateDps: Float,
    ): VisualOccupancySummary {
        val isTurning = abs(yawRateDps) > 18.0f || motionState == MotionState.TURNING_LEFT || motionState == MotionState.TURNING_RIGHT
        val decay = if (isTurning) turnDecayFactor else decayFactorPerFrame

        // 1. Apply temporal decay to existing grid
        for (c in cells) {
            c.confidence *= decay
            if (c.confidence < minConfidenceThreshold) {
                c.state = OccupancyCellState.UNCERTAIN
                c.occupancyProbability = 0.5f
            }
        }

        // 2. Integrate fresh visual evidence per sector
        for (ev in evidenceList) {
            if (!ev.isFresh || ev.confidence < minConfidenceThreshold) continue

            val sectorCells = cells.filter { it.sector == ev.sector }
            val isHazard = ev.risk == SectorRisk.CRITICAL || ev.risk == SectorRisk.WARNING ||
                    (ev.minTtcSec != null && ev.minTtcSec <= 2.5f)

            for (c in sectorCells) {
                if (isHazard) {
                    // Hazard detected: mark cells near the clearance horizon as OCCUPIED
                    val hazardDist = ev.clearanceDistanceM
                    if (hazardDist in c.minDistanceM..c.maxDistanceM || (hazardDist < c.minDistanceM && c.ringIndex == 0)) {
                        c.state = OccupancyCellState.OCCUPIED
                        c.occupancyProbability = (0.5f + 0.45f * ev.confidence).coerceAtMost(0.95f)
                        c.confidence = (c.confidence * 0.5f + ev.confidence * 0.5f).coerceIn(0f, 1f)
                    } else if (c.maxDistanceM < hazardDist) {
                        // Ground before the hazard is traversable
                        c.state = OccupancyCellState.FREE
                        c.occupancyProbability = (0.5f - 0.4f * ev.confidence).coerceAtLeast(0.05f)
                        c.confidence = (c.confidence * 0.5f + ev.confidence * 0.5f).coerceIn(0f, 1f)
                    }
                } else if (ev.risk == SectorRisk.SAFE || ev.risk == SectorRisk.CAUTION) {
                    // Traversable corridor up to clearance distance
                    if (c.minDistanceM < ev.clearanceDistanceM) {
                        c.state = OccupancyCellState.FREE
                        c.occupancyProbability = (0.5f - 0.4f * ev.confidence).coerceAtLeast(0.05f)
                        c.confidence = (c.confidence * 0.5f + ev.confidence * 0.5f).coerceIn(0f, 1f)
                    }
                }
            }
        }

        // 3. Compile summary metrics
        var occupiedCount = 0
        var freeCount = 0
        var uncertainCount = 0
        var nearestHazardM: Float? = null

        for (c in cells) {
            when (c.state) {
                OccupancyCellState.OCCUPIED -> {
                    occupiedCount++
                    val midDist = (c.minDistanceM + c.maxDistanceM) / 2.0f
                    if (nearestHazardM == null || midDist < nearestHazardM) {
                        nearestHazardM = midDist
                    }
                }
                OccupancyCellState.FREE -> freeCount++
                OccupancyCellState.UNCERTAIN -> uncertainCount++
            }
        }

        val centerCells = cells.filter { it.sector == CorridorSector.CENTER && it.state == OccupancyCellState.FREE }
        val centerClearance = centerCells.map { it.maxDistanceM }.maxOrNull() ?: 0.0f

        return VisualOccupancySummary(
            occupiedCellsCount = occupiedCount,
            freeCellsCount = freeCount,
            uncertainCellsCount = uncertainCount,
            nearestHazardDistanceM = nearestHazardM,
            centerClearanceM = centerClearance,
        )
    }
}
