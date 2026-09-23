package com.minesafety.roboeye.vision

import com.minesafety.roboeye.core.model.CameraFrame
import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test
import java.nio.ByteBuffer

class FastFeatureDetectorTest {

    private fun makeSyntheticFrame(width: Int = 100, height: Int = 100): CameraFrame {
        val buf = ByteBuffer.allocate(width * height)
        // High-contrast solid box in center
        for (y in 0 until height) {
            for (x in 0 until width) {
                val value = if (x in 35..65 && y in 35..65) 240.toByte() else 20.toByte()
                buf.put(value)
            }
        }
        buf.flip()
        return CameraFrame(
            sequenceNumber = 1L,
            timestampNs = 1_000_000_000L,
            width = width,
            height = height,
            yBuffer = buf,
            yRowStride = width,
            yPixelStride = 1,
        )
    }

    @Test
    fun detect_findsFastCorners() {
        val frame = makeSyntheticFrame(100, 100)
        val detector = FastFeatureDetector(threshold = 20, sampleStep = 1, borderMargin = 5)
        val features = detector.detect(frame, maxFeatures = 20)

        assertTrue("Expected to detect FAST corners on box, found ${features.size}", features.isNotEmpty())
        for (f in features) {
            assertTrue("Corner X ${f.x} must be within bounds", f.x in 5.0f..95.0f)
            assertTrue("Corner Y ${f.y} must be within bounds", f.y in 5.0f..95.0f)
        }
    }

    @Test
    fun detect_flatImage_returnsZeroFeatures() {
        val width = 100
        val height = 100
        val buf = ByteBuffer.allocate(width * height)
        for (i in 0 until width * height) buf.put(128.toByte())
        buf.flip()

        val frame = CameraFrame(1L, 1L, width, height, yBuffer = buf)
        val detector = FastFeatureDetector()
        val features = detector.detect(frame, maxFeatures = 20)

        assertTrue("Flat patch should yield 0 features", features.isEmpty())
    }

    @Test
    fun detect_respectsMaxQuota() {
        val frame = makeSyntheticFrame(200, 200)
        val detector = FastFeatureDetector(threshold = 15, sampleStep = 1)
        val features = detector.detect(frame, maxFeatures = 8)

        assertTrue("Should not exceed max features quota", features.size <= 8)
    }
}
