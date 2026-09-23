package com.minesafety.roboeye.mapping

import com.minesafety.roboeye.fusion.FusedWorldState

data class OccupancyCell(val x: Int, val y: Int, val occupancyProbability: Float)

/**
 * 2D Local Costmap & Occupancy Grid mapping boundary interface.
 */
interface MappingEngine {
    /** Clears the local map. */
    fun clear()

    /** Integrates a fused world observation into the occupancy grid. */
    fun updateMap(world: FusedWorldState)
}
