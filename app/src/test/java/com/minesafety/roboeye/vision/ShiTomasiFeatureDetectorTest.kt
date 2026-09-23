package com.minesafety.roboeye.vision

import com.minesafety.roboeye.core.model.CameraFrame
import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test
import java.nio.ByteBuffer

class ShiTomasiFeatureDetectorTest {

    private fun createSyntheticFrame(width: Int = 100, height: Int = 100): Pair<CameraFrame, ByteBuffer> {
        val buf = ByteBuffer.allocate(width * height)
        // Draw high contrast box with clear corners in center
        for (y in 0 until height) {
            for (x in 0 until width) {
                val value = if (x in 30..70 && y in 30..70) 240.toByte() else 16.toByte()
                buf.put(value)
            }
        }
        buf.flip()
        val frame = CameraFrame(
            sequenceNumber = 1L,
            timestampNs = 1_000_000_000L,
            width = width,
            height = height,
            yBuffer = buf,
            yRowStride = width,
            yPixelStride = 1,
        )
        return Pair(frame, buf)
    }

    @Test
    fun detect_findsCornersInSyntheticHighContrastPattern() {
        val (frame, _) = createSyntheticFrame(100, 100)
        val detector = ShiTomasiFeatureDetector(minQuality = 0.01f, sampleStep = 1, borderMargin = 5)
        val features = detector.detect(frame, maxFeatures = 20)

        assertTrue("Expected to detect corners, found ${features.size}", features.isNotEmpty())
        for (f in features) {
            assertTrue("Corner X ${f.x} must be within bounds", f.x in 5.0f..95.0f)
            assertTrue("Corner Y ${f.y} must be within bounds", f.y in 5.0f..95.0f)
            assertTrue("Response must be positive", f.response > 0f)
        }
    }

    @Test
    fun detect_flatImage_returnsEmpty() {
        val width = 100
        val height = 100
        val buf = ByteBuffer.allocate(width * height)
        for (i in 0 until width * height) {
            buf.put(128.toByte())
        }
        buf.flip()

        val frame = CameraFrame(
            sequenceNumber = 1L,
            timestampNs = 1_000_000_000L,
            width = width,
            height = height,
            yBuffer = buf,
            yRowStride = width,
            yPixelStride = 1,
        )

        val detector = ShiTomasiFeatureDetector()
        val features = detector.detect(frame, maxFeatures = 20)
        assertTrue("Flat image should yield 0 features", features.isEmpty())
    }

    @Test
    fun detect_respectsMaxFeaturesLimit() {
        val (frame, _) = createSyntheticFrame(200, 200)
        val detector = ShiTomasiFeatureDetector(sampleStep = 1)
        val features = detector.detect(frame, maxFeatures = 5)

        assertTrue("Should not exceed max features", features.size <= 5)
    }
}
