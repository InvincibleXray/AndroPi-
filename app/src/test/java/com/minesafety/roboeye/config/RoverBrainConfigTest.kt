package com.minesafety.roboeye.config

import com.minesafety.roboeye.core.config.PerformanceProfile
import com.minesafety.roboeye.core.config.RoverBrainConfig
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Test

class RoverBrainConfigTest {

    @Test
    fun defaultProfile_isLowEndForRedmi6A() {
        val config = RoverBrainConfig()
        assertEquals(PerformanceProfile.LOW_END, config.profile)

        // Verifies optimization requirements for Redmi 6A
        val profile = config.profile
        assertEquals(10.0f, profile.maxCameraFps, 0.001f)
        assertEquals(640, profile.cameraWidth)
        assertEquals(360, profile.cameraHeight)
        assertFalse(profile.enableCameraPreviewByDefault)
        assertFalse(profile.enableYoloByDefault)
        assertTrue(profile.maxVisionHz <= 10.0f)
        assertTrue(profile.bufferPoolSize <= 3)
    }

    @Test
    fun safetyThresholds_matchBaselineRequirements() {
        val config = RoverBrainConfig()
        assertEquals(0.5f, config.eStopDistanceM, 0.001f)
        assertEquals(1.5f, config.slowDownDistanceM, 0.001f)
        assertEquals(25.0f, config.maxTiltDeg, 0.001f)
        assertEquals(0.10f, config.minVisibilityScore, 0.001f)
    }
}
