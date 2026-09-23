package com.minesafety.roboeye.mapping

import com.minesafety.roboeye.fusion.CorridorSector
import com.minesafety.roboeye.fusion.Point2D
import com.minesafety.roboeye.localization.LocalPose
import com.minesafety.roboeye.vision.OccupancyCell
import com.minesafety.roboeye.vision.OccupancyCellState
import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Before
import org.junit.Test

class LocalSpatialMapTest {

    private lateinit var transformer: MapCoordinateTransformer
    private lateinit var spatialMap: LocalSpatialMap

    @Before
    fun setUp() {
        transformer = MapCoordinateTransformer(
            gridResolutionM = 0.25f,
            gridWidthCells = 24,
            gridHeightCells = 24,
        )
        spatialMap = LocalSpatialMap(
            gridWidth = 24,
            gridHeight = 24,
            resolutionM = 0.25f,
            confidenceDecayPerSec = 0.80f,
        )
    }

    @Test
    fun rigidBodyTransform_transformsPolarSectorToMapCoordinates() {
        // Center sector, ring 0 (r = 0.45m)
        val bodyPt = transformer.polarToBody(CorridorSector.CENTER, ringIndex = 0)
        assertEquals(0.45f, bodyPt.xM, 0.05f)
        assertEquals(0.0f, bodyPt.yM, 0.05f)

        // Rover is at (1.0m, 2.0m) heading 90 deg (facing left / +Y)
        val roverPose = LocalPose(xM = 1.0f, yM = 2.0f, yawDeg = 90.0f)
        val mapPt = transformer.bodyToMap(bodyPt, roverPose)

        // Since yaw is 90 deg, forward body motion extends in +Y world direction
        assertEquals(1.0f, mapPt.xM, 0.05f)
        assertEquals(2.45f, mapPt.yM, 0.05f)
    }

    @Test
    fun mapUpdate_integratesOccupancyCells() {
        val mapper = LocalSpatialMapper(spatialMap = spatialMap, transformer = transformer)
        val cells = listOf(
            OccupancyCell(
                sector = CorridorSector.CENTER,
                ringIndex = 0,
                minDistanceM = 0.2f,
                maxDistanceM = 0.7f,
                state = OccupancyCellState.FREE,
                confidence = 0.9f,
            ),
            OccupancyCell(
                sector = CorridorSector.CENTER,
                ringIndex = 1,
                minDistanceM = 0.7f,
                maxDistanceM = 1.5f,
                state = OccupancyCellState.OCCUPIED,
                confidence = 0.85f,
            ),
        )

        val summary = mapper.updateFromVision(
            occupancyCells = cells,
            roverPose = LocalPose.ORIGIN,
            landmarks = emptyList(),
            timestampNs = 1000L,
        )

        assertTrue(summary.freeCellsCount >= 1)
        assertTrue(summary.occupiedCellsCount >= 1)
    }

    @Test
    fun temporalDecay_reducesConfidence_revertsToUncertain() {
        val now = 1_000_000L
        spatialMap.lastDecayTimeMs = now
        spatialMap.updateCell(12, 14, MapCellState.OCCUPIED, confidence = 0.30f, timestampMs = now)
        assertEquals(MapCellState.OCCUPIED, spatialMap.cells[12][14].state)

        // Advance time by 3 seconds: 0.30 * (0.80)^3 = 0.30 * 0.512 = 0.153 (< 0.20 threshold)
        spatialMap.decay(now + 3000L)

        // Reverts to UNCERTAIN ("Unknown Is NOT Safe")
        assertEquals(MapCellState.UNCERTAIN, spatialMap.cells[12][14].state)
        assertEquals(0.0f, spatialMap.cells[12][14].confidence, 0.001f)
    }

    @Test
    fun mapMemoryBound_fixed24x24Array() {
        assertEquals(24, spatialMap.cells.size)
        assertEquals(24, spatialMap.cells[0].size)
    }

    @Test
    fun mapClear_resetsAllCellsToUncertain() {
        spatialMap.updateCell(10, 10, MapCellState.FREE, 0.9f, 1000L)
        spatialMap.updateCell(15, 15, MapCellState.OCCUPIED, 0.8f, 1000L)

        spatialMap.clear()

        val summary = spatialMap.getSummary()
        assertEquals(0, summary.freeCellsCount)
        assertEquals(0, summary.occupiedCellsCount)
        assertEquals(24 * 24, summary.uncertainCellsCount)
    }

    @Test
    fun cameraMountingGeometry_translatesPolarSectorCorrectly() {
        // Left sector has positive Y (+Y left)
        val leftBody = transformer.polarToBody(CorridorSector.LEFT, ringIndex = 1)
        assertTrue(leftBody.yM > 0.0f)

        // Right sector has negative Y (-Y right)
        val rightBody = transformer.polarToBody(CorridorSector.RIGHT, ringIndex = 1)
        assertTrue(rightBody.yM < 0.0f)
    }
}
