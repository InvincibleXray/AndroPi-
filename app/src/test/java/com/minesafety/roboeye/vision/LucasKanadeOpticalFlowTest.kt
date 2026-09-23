package com.minesafety.roboeye.vision

import com.minesafety.roboeye.core.model.CameraFrame
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Test
import java.nio.ByteBuffer
import kotlin.math.abs

class LucasKanadeOpticalFlowTest {

    private fun makeBoxFrame(width: Int, height: Int, boxX: Int, boxY: Int, boxSize: Int): CameraFrame {
        val buf = ByteBuffer.allocate(width * height)
        for (y in 0 until height) {
            for (x in 0 until width) {
                val inBox = (x in boxX until (boxX + boxSize)) && (y in boxY until (boxY + boxSize))
                buf.put(if (inBox) 230.toByte() else 30.toByte())
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
    fun track_recoversKnownTranslation() {
        val width = 120
        val height = 120
        val shiftX = 2
        val shiftY = 1

        // Frame 1: Box at (40, 40)
        val frame1 = makeBoxFrame(width, height, boxX = 40, boxY = 40, boxSize = 30)
        // Frame 2: Box shifted by (+2, +1) -> (42, 41)
        val frame2 = makeBoxFrame(width, height, boxX = 40 + shiftX, boxY = 40 + shiftY, boxSize = 30)

        // Feature corner at (40, 40)
        val pt = FeaturePoint(40f, 40f, 100f)
        val tracker = LucasKanadeOpticalFlow(windowRadius = 4, maxIterations = 8)
        val flow = tracker.track(frame1, frame2, listOf(pt))

        assertEquals(1, flow.size)
        val vec = flow[0]
        assertTrue("Flow vector must be marked valid", vec.isValid)
        assertTrue("dx must be close to $shiftX, was ${vec.dx}", abs(vec.dx - shiftX) < 1.0f)
        assertTrue("dy must be close to $shiftY, was ${vec.dy}", abs(vec.dy - shiftY) < 1.0f)
    }

    @Test
    fun track_flatRegion_rejectsDueToSingularity() {
        val width = 100
        val height = 100
        val buf1 = ByteBuffer.allocate(width * height)
        val buf2 = ByteBuffer.allocate(width * height)
        for (i in 0 until width * height) {
            buf1.put(100.toByte())
            buf2.put(100.toByte())
        }
        buf1.flip()
        buf2.flip()

        val f1 = CameraFrame(1L, 1L, width, height, yBuffer = buf1)
        val f2 = CameraFrame(2L, 2L, width, height, yBuffer = buf2)

        val tracker = LucasKanadeOpticalFlow()
        val flow = tracker.track(f1, f2, listOf(FeaturePoint(50f, 50f)))

        assertEquals(1, flow.size)
        assertFalse("Flat patch should be marked invalid", flow[0].isValid)
    }
}
