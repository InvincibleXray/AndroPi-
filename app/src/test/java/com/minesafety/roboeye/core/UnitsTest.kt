package com.minesafety.roboeye.core

import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test

class UnitsTest {

    @Test
    fun mps2ToG_convertsAccurately() {
        assertEquals(1.0f, Units.mps2ToG(9.80665f), 0.001f)
        assertEquals(0.0f, Units.mps2ToG(0.0f), 0.001f)
        assertEquals(2.0f, Units.mps2ToG(19.6133f), 0.001f)
    }

    @Test
    fun tiltDeg_computesMaxTiltAngle() {
        assertEquals(15.0f, Units.tiltDeg(15.0f, 10.0f), 0.001f)
        assertEquals(20.0f, Units.tiltDeg(5.0f, -20.0f), 0.001f)
        assertEquals(0.0f, Units.tiltDeg(0.0f, 0.0f), 0.001f)
    }

    @Test
    fun accelMagnitudeG_computesPythagoreanDistance() {
        assertEquals(5.0f, Units.accelMagnitudeG(3.0f, 4.0f), 0.001f)
        assertEquals(0.0f, Units.accelMagnitudeG(0.0f, 0.0f), 0.001f)
    }

    @Test
    fun applyOffset_calibratesZeroPoint() {
        // Raw tilt 90° with offset 90° -> calibrated 0°
        assertEquals(0.0f, Units.applyOffset(90.0f, 90.0f), 0.001f)
        // Raw tilt 95° with offset 90° -> calibrated +5°
        assertEquals(5.0f, Units.applyOffset(95.0f, 90.0f), 0.001f)
    }

    @Test
    fun clamp01_restrictsBounds() {
        assertEquals(0.0f, Units.clamp01(-0.5f), 0.001f)
        assertEquals(1.0f, Units.clamp01(1.5f), 0.001f)
        assertEquals(0.6f, Units.clamp01(0.6f), 0.001f)
    }
}
