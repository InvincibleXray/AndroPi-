package com.minesafety.roboeye.mapping

import com.minesafety.roboeye.localization.Landmark
import com.minesafety.roboeye.localization.LocalPose

/**
 * State of a single spatial grid cell in the local map.
 */
enum class MapCellState {
    /** Verified clear ground corridor. */
    FREE,
    /** Visual obstacle / hazard boundary. */
    OCCUPIED,
    /** Uninstrumented, low-feature, or decayed region ("Unknown Is NOT Safe"). */
    UNCERTAIN,
}

/**
 * Single cell in the bounded 2D local spatial grid.
 */
data class LocalMapCell(
    val gridX: Int,
    val gridY: Int,
    val mapXM: Float,
    val mapYM: Float,
    var state: MapCellState = MapCellState.UNCERTAIN,
    var confidence: Float = 0.0f,
    var lastUpdatedMs: Long = 0L,
)

/**
 * Keyframe capturing robot pose, associated visual landmarks, and timestamp.
 */
data class Keyframe(
    val id: Int,
    val pose: LocalPose,
    val landmarks: List<Landmark>,
    val timestampNs: Long,
)

/**
 * Summary telemetry of the persistent local map.
 */
data class LocalMapState(
    val timestampMs: Long = System.currentTimeMillis(),
    val activeLandmarksCount: Int = 0,
    val keyframesCount: Int = 0,
    val occupiedCellsCount: Int = 0,
    val freeCellsCount: Int = 0,
    val uncertainCellsCount: Int = 0,
    val mapConfidence: Float = 0.0f,
    val isRelocalized: Boolean = false,
) {
    fun toSummaryString(): String {
        return "Map: KF=$keyframesCount, LM=$activeLandmarksCount, Occ=$occupiedCellsCount, Free=$freeCellsCount [Conf: ${(mapConfidence * 100).toInt()}%]"
    }
}
