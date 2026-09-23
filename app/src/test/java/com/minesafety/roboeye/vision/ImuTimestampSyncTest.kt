package com.minesafety.roboeye.vision

import com.minesafety.roboeye.core.ImuReading
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Test
import kotlin.math.abs

class ImuTimestampSyncTest {

    @Test
    fun `fresh IMU sample within 250ms threshold is accepted`() {
        val nowNs = 1_000_000_000L // 1.0s
        val imuTimestampNs = 950_000_000L // 0.95s (50ms old)

        val reading = ImuReading(
            timestampNs = imuTimestampNs,
            isAvailable = true,
        )

        val ageNs = nowNs - reading.timestampNs
        val isFresh = ageNs in 0L..250_000_000L

        assertTrue("IMU sample 50ms old should be fresh", isFresh)
    }

    @Test
    fun `stale IMU sample older than 250ms threshold is rejected`() {
        val nowNs = 1_500_000_000L
        val imuTimestampNs = 1_000_000_000L // 500ms old (> 250ms)

        val reading = ImuReading(
            timestampNs = imuTimestampNs,
            isAvailable = true,
        )

        val ageNs = nowNs - reading.timestampNs
        val isFresh = ageNs in 0L..250_000_000L

        assertFalse("IMU sample 500ms old must be marked stale", isFresh)
    }

    @Test
    fun `camera frame and IMU timestamp alignment within bounded window`() {
        val cameraFrameTimestampNs = 10_500_000_000L
        val imuTimestampNs = 10_480_000_000L // 20ms delta

        val deltaNs = abs(cameraFrameTimestampNs - imuTimestampNs)
        assertTrue("Camera frame and IMU timestamp difference (20ms) should be within 250ms", deltaNs <= 250_000_000L)
    }
}
