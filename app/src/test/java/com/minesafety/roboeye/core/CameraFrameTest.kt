package com.minesafety.roboeye.core

import com.minesafety.roboeye.core.model.CameraFrame
import org.junit.Assert.assertEquals
import org.junit.Assert.assertNull
import org.junit.Test

class CameraFrameTest {

    @Test
    fun cameraFrame_storesGeometryAndFormat() {
        val frame = CameraFrame(
            sequenceNumber = 42L,
            timestampNs = 1_000_000_000L,
            width = 640,
            height = 480,
            rotationDegrees = 90,
            format = CameraFrame.FrameFormat.YUV_420_888,
        )

        assertEquals(42L, frame.sequenceNumber)
        assertEquals(1_000_000_000L, frame.timestampNs)
        assertEquals(640, frame.width)
        assertEquals(480, frame.height)
        assertEquals(90, frame.rotationDegrees)
        assertEquals(CameraFrame.FrameFormat.YUV_420_888, frame.format)
        assertNull(frame.jpegData)
    }
}
