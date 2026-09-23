package com.minesafety.roboeye.mapping

import com.minesafety.roboeye.localization.Landmark
import com.minesafety.roboeye.localization.LocalPose
import kotlin.math.hypot

/**
 * Bounded 2D spatial grid map with temporal decay and bounded keyframe storage.
 * Designed for constant memory footprint and zero heap allocation on the Cortex-A53 hot path.
 */
class LocalSpatialMap(
    val gridWidth: Int = 24,
    val gridHeight: Int = 24,
    val resolutionM: Float = 0.25f,
    private val maxKeyframes: Int = 12,
    private val confidenceDecayPerSec: Float = 0.92f,
    private val minConfidenceThreshold: Float = 0.20f,
) {
    // 576 pre-allocated cells
    val cells: Array<Array<LocalMapCell>> = Array(gridWidth) { gx ->
        Array(gridHeight) { gy ->
            val xm = (gx - gridWidth / 2) * resolutionM
            val ym = (gy - gridHeight / 2) * resolutionM
            LocalMapCell(
                gridX = gx,
                gridY = gy,
                mapXM = xm,
                mapYM = ym,
                state = MapCellState.UNCERTAIN,
                confidence = 0.0f,
                lastUpdatedMs = 0L,
            )
        }
    }

    private val keyframes = mutableListOf<Keyframe>()
    val storedKeyframes: List<Keyframe> get() = keyframes

    private var nextKeyframeId = 1
    var lastDecayTimeMs: Long = 0L

    /**
     * Updates an individual cell with fresh observation.
     */
    fun updateCell(gridX: Int, gridY: Int, state: MapCellState, confidence: Float, timestampMs: Long) {
        if (gridX !in 0 until gridWidth || gridY !in 0 until gridHeight) return

        val cell = cells[gridX][gridY]
        if (cell.state == MapCellState.UNCERTAIN) {
            cell.state = state
            cell.confidence = confidence.coerceIn(0f, 1f)
        } else {
            if (cell.state == state) {
                // Reinforce observation
                cell.confidence = (cell.confidence + confidence * 0.25f).coerceIn(0f, 1f)
            } else if (state == MapCellState.OCCUPIED) {
                // Safety conservative bias: Occupied evidence overrides Free if confident
                if (confidence >= 0.40f) {
                    cell.state = MapCellState.OCCUPIED
                    cell.confidence = confidence
                } else {
                    cell.confidence = (cell.confidence - confidence * 0.3f).coerceAtLeast(0f)
                    if (cell.confidence < minConfidenceThreshold) {
                        cell.state = MapCellState.UNCERTAIN
                    }
                }
            } else if (state == MapCellState.FREE) {
                // Free evidence cautiously decrements occupied confidence
                cell.confidence = (cell.confidence - confidence * 0.25f).coerceAtLeast(0f)
                if (cell.confidence < minConfidenceThreshold) {
                    cell.state = MapCellState.FREE
                    cell.confidence = confidence * 0.5f
                }
            } else {
                cell.confidence = (cell.confidence * 0.5f + confidence * 0.5f).coerceIn(0f, 1f)
            }
        }
        cell.lastUpdatedMs = timestampMs
    }

    /**
     * Applies temporal decay across all cells.
     */
    fun decay(nowMs: Long) {
        if (lastDecayTimeMs <= 0L) {
            lastDecayTimeMs = nowMs
            return
        }
        val dtSec = ((nowMs - lastDecayTimeMs) / 1000.0f).coerceIn(0.0f, 10.0f)
        lastDecayTimeMs = nowMs
        if (dtSec <= 0f) return

        val decayFactor = Math.pow(confidenceDecayPerSec.toDouble(), dtSec.toDouble()).toFloat()

        for (x in 0 until gridWidth) {
            for (y in 0 until gridHeight) {
                val cell = cells[x][y]
                if (cell.state != MapCellState.UNCERTAIN) {
                    cell.confidence *= decayFactor
                    if (cell.confidence < minConfidenceThreshold) {
                        cell.state = MapCellState.UNCERTAIN
                        cell.confidence = 0.0f
                    }
                }
            }
        }
    }

    /**
     * Considers creating a keyframe if rover has moved sufficiently since last keyframe.
     */
    fun maybeCreateKeyframe(
        pose: LocalPose,
        landmarks: List<Landmark>,
        timestampNs: Long,
        minTransM: Float = 0.40f,
        minRotDeg: Float = 20.0f,
    ): Boolean {
        val lastKf = keyframes.lastOrNull()
        val shouldCreate = if (lastKf == null) {
            true
        } else {
            val dist = hypot(pose.xM - lastKf.pose.xM.toDouble(), pose.yM - lastKf.pose.yM.toDouble()).toFloat()
            val rotDiff = Math.abs(pose.yawDeg - lastKf.pose.yawDeg)
            dist >= minTransM || rotDiff >= minRotDeg
        }

        if (shouldCreate) {
            if (keyframes.size >= maxKeyframes) {
                keyframes.removeAt(0) // Enforce bounded memory
            }
            keyframes.add(
                Keyframe(
                    id = nextKeyframeId++,
                    pose = pose,
                    landmarks = landmarks.map { it.copy() },
                    timestampNs = timestampNs,
                )
            )
            return true
        }
        return false
    }

    /**
     * Summarizes current local map occupancy.
     */
    fun getSummary(nowMs: Long = System.currentTimeMillis(), isRelocalized: Boolean = false): LocalMapState {
        var occupied = 0
        var free = 0
        var uncertain = 0
        var totalConf = 0.0f
        var nonUncertainCount = 0

        for (x in 0 until gridWidth) {
            for (y in 0 until gridHeight) {
                val cell = cells[x][y]
                when (cell.state) {
                    MapCellState.OCCUPIED -> {
                        occupied++
                        totalConf += cell.confidence
                        nonUncertainCount++
                    }
                    MapCellState.FREE -> {
                        free++
                        totalConf += cell.confidence
                        nonUncertainCount++
                    }
                    MapCellState.UNCERTAIN -> uncertain++
                }
            }
        }

        val mapConf = if (nonUncertainCount > 0) (totalConf / nonUncertainCount).coerceIn(0f, 1f) else 0.0f

        return LocalMapState(
            timestampMs = nowMs,
            activeLandmarksCount = keyframes.lastOrNull()?.landmarks?.size ?: 0,
            keyframesCount = keyframes.size,
            occupiedCellsCount = occupied,
            freeCellsCount = free,
            uncertainCellsCount = uncertain,
            mapConfidence = mapConf,
            isRelocalized = isRelocalized,
        )
    }

    fun clear() {
        for (x in 0 until gridWidth) {
            for (y in 0 until gridHeight) {
                cells[x][y].state = MapCellState.UNCERTAIN
                cells[x][y].confidence = 0.0f
                cells[x][y].lastUpdatedMs = 0L
            }
        }
        keyframes.clear()
        nextKeyframeId = 1
        lastDecayTimeMs = 0L
    }
}
