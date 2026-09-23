package com.minesafety.roboeye.mapping

import com.minesafety.roboeye.fusion.FusedWorldState
import com.minesafety.roboeye.localization.Landmark
import com.minesafety.roboeye.localization.LocalPose
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow

/**
 * Concrete local spatial mapper implementing [MappingEngine].
 * Transforms Phase 4 egocentric visual occupancy observations into the persistent
 * local map frame anchored to the rover's startup origin.
 */
class LocalSpatialMapper(
    val spatialMap: LocalSpatialMap = LocalSpatialMap(),
    val transformer: MapCoordinateTransformer = MapCoordinateTransformer(),
) : MappingEngine {

    private val _mapStateFlow = MutableStateFlow(LocalMapState())
    val mapStateFlow: StateFlow<LocalMapState> = _mapStateFlow.asStateFlow()

    /**
     * Integrates Phase 4 visual occupancy grid cells and rover pose into the persistent local map.
     */
    fun updateFromVision(
        occupancyCells: List<com.minesafety.roboeye.vision.OccupancyCell>,
        roverPose: LocalPose,
        landmarks: List<Landmark>,
        timestampNs: Long,
        isRelocalized: Boolean = false,
    ): LocalMapState {
        val nowMs = System.currentTimeMillis()

        // 1. Transform each Phase 4 egocentric polar cell to persistent local map frame
        for (i in occupancyCells.indices) {
            val cell = occupancyCells[i]
            val bodyPt = transformer.polarToBody(cell.sector, cell.ringIndex)
            val mapPt = transformer.bodyToMap(bodyPt, roverPose)
            val gridCoords = transformer.mapToGrid(mapPt)
            if (gridCoords != null) {
                val (gx, gy) = gridCoords
                val mapState = transformer.mapCellState(cell.state)
                spatialMap.updateCell(
                    gridX = gx,
                    gridY = gy,
                    state = mapState,
                    confidence = cell.confidence,
                    timestampMs = nowMs,
                )
            }
        }

        // 2. Apply temporal decay to fade stale dynamic evidence
        spatialMap.decay(nowMs)

        // 3. Register keyframe if rover moved sufficiently
        spatialMap.maybeCreateKeyframe(
            pose = roverPose,
            landmarks = landmarks,
            timestampNs = timestampNs,
        )

        // 4. Update summary state flow
        val summary = spatialMap.getSummary(nowMs, isRelocalized)
        _mapStateFlow.value = summary
        return summary
    }

    override fun updateMap(world: FusedWorldState) {
        // Multi-sensor regression compatibility stub
        spatialMap.decay(System.currentTimeMillis())
        _mapStateFlow.value = spatialMap.getSummary(System.currentTimeMillis())
    }

    override fun clear() {
        spatialMap.clear()
        _mapStateFlow.value = LocalMapState()
    }
}
