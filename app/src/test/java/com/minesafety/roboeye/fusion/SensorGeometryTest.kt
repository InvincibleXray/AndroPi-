package com.minesafety.roboeye.fusion

import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Test
import kotlin.math.abs

class SensorGeometryTest {

    private val footprint = RoverFootprint(widthM = 0.35f, lengthM = 0.45f, safetyMarginM = 0.15f)

    @Test
    fun project_frontSensor_projectsAlongXAxis() {
        val mount = DistanceSensorMount.DEFAULT_FRONT // x=0.225, y=0.0, yaw=0
        val point = mount.project(1.0f)

        assertEquals(1.225f, point.xM, 0.001f)
        assertEquals(0.0f, point.yM, 0.001f)
        assertEquals(CorridorSector.CENTER, footprint.classifySector(point))
    }

    @Test
    fun project_leftSensor_projectsDiagonalLeft() {
        val mount = DistanceSensorMount.DEFAULT_LEFT // x=0.150, y=0.175, yaw=45°
        val point = mount.project(1.0f)

        val expectedX = 0.150f + 1.0f * kotlin.math.cos(Math.toRadians(45.0)).toFloat()
        val expectedY = 0.175f + 1.0f * kotlin.math.sin(Math.toRadians(45.0)).toFloat()

        assertTrue("X coordinate must match projection", abs(point.xM - expectedX) < 0.005f)
        assertTrue("Y coordinate must match projection", abs(point.yM - expectedY) < 0.005f)
        assertEquals(CorridorSector.LEFT, footprint.classifySector(point))
    }

    @Test
    fun footprint_detectsCollisionZone() {
        // Point right in front of bumper within safety margin
        val nearPoint = Point2D(0.30f, 0.05f)
        assertTrue("Point inside boundary should be within footprint", footprint.isWithinFootprint(nearPoint))

        // Point far away
        val farPoint = Point2D(2.0f, 0.0f)
        assertFalse("Far point should not be within footprint", footprint.isWithinFootprint(farPoint))

        // Point laterally outside
        val widePoint = Point2D(0.30f, 0.60f)
        assertFalse("Wide point should not be within footprint", footprint.isWithinFootprint(widePoint))
    }
}
