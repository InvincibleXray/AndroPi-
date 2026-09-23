package com.minesafety.roboeye.vision

import com.minesafety.roboeye.fusion.CorridorSector
import com.minesafety.roboeye.fusion.VisualFreeSpaceEvidence
import org.junit.Assert.assertEquals
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertTrue
import org.junit.Before
import org.junit.Test

class LocalVisualOccupancyGridTest {

    private lateinit var grid: LocalVisualOccupancyGrid

    @Before
    fun setUp() {
        grid = LocalVisualOccupancyGrid()
    }

    @Test
    fun `initial grid has all 12 cells in UNCERTAIN state`() {
        assertEquals(12, grid.cells.size)
        assertTrue(grid.cells.all { it.state == OccupancyCellState.UNCERTAIN })
    }

    @Test
    fun `hazard expansion marks occupied cells at appropriate distance`() {
        // Critical obstacle in center corridor at 1.0m (Ring 1: 0.7m..1.5m)
        val centerHazard = VisualFreeSpaceEvidence(
            sector = CorridorSector.CENTER,
            confidence = 0.9f,
            isFresh = true,
            clearanceDistanceM = 1.0f,
            risk = SectorRisk.CRITICAL,
            featureCount = 20,
            isDivergent = true,
            minTtcSec = 1.4f,
        )

        val summary = grid.update(listOf(centerHazard), MotionState.FORWARD_TRANSLATION, yawRateDps = 0.0f)

        assertTrue("Should have at least 1 occupied cell", summary.occupiedCellsCount > 0)
        assertNotNull("Nearest hazard distance should be populated", summary.nearestHazardDistanceM)
        assertTrue("Hazard distance should be near 1.0m", summary.nearestHazardDistanceM!! in 0.5f..1.5f)

        // Ring 1 in center sector should be OCCUPIED
        val ring1Center = grid.cells.first { it.sector == CorridorSector.CENTER && it.ringIndex == 1 }
        assertEquals(OccupancyCellState.OCCUPIED, ring1Center.state)

        // Ring 0 (0.2m..0.7m) in center sector should be FREE (space before obstacle)
        val ring0Center = grid.cells.first { it.sector == CorridorSector.CENTER && it.ringIndex == 0 }
        assertEquals(OccupancyCellState.FREE, ring0Center.state)
    }

    @Test
    fun `unreinforced cells decay back to UNCERTAIN state over time`() {
        val centerClear = VisualFreeSpaceEvidence(
            sector = CorridorSector.CENTER,
            confidence = 0.8f,
            isFresh = true,
            clearanceDistanceM = 3.5f,
            risk = SectorRisk.SAFE,
            featureCount = 20,
            isDivergent = true,
        )

        // Frame 1: populate grid with clear ground
        grid.update(listOf(centerClear), MotionState.FORWARD_TRANSLATION, 0.0f)
        val centerRing0 = grid.cells.first { it.sector == CorridorSector.CENTER && it.ringIndex == 0 }
        assertEquals(OccupancyCellState.FREE, centerRing0.state)

        // Run 15 update cycles with no fresh evidence -> cell must decay back to UNCERTAIN
        for (i in 0 until 15) {
            grid.update(emptyList(), MotionState.STATIONARY, 0.0f)
        }

        assertEquals(OccupancyCellState.UNCERTAIN, centerRing0.state)
    }

    @Test
    fun `high turning yaw rate accelerates uncertainty decay`() {
        val centerClear = VisualFreeSpaceEvidence(
            sector = CorridorSector.CENTER,
            confidence = 0.8f,
            isFresh = true,
            clearanceDistanceM = 3.5f,
            risk = SectorRisk.SAFE,
            featureCount = 20,
            isDivergent = true,
        )
        grid.update(listOf(centerClear), MotionState.FORWARD_TRANSLATION, 0.0f)

        // A single rapid turn maneuver (yaw rate 35 deg/s) should heavily accelerate decay
        for (i in 0 until 4) {
            grid.update(emptyList(), MotionState.TURNING_LEFT, yawRateDps = 35.0f)
        }

        val centerRing0 = grid.cells.first { it.sector == CorridorSector.CENTER && it.ringIndex == 0 }
        assertEquals(OccupancyCellState.UNCERTAIN, centerRing0.state)
    }
}
