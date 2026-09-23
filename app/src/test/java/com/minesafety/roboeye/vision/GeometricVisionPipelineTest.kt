package com.minesafety.roboeye.vision

import com.minesafety.roboeye.core.model.CameraFrame
import org.junit.Assert.assertEquals
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertTrue
import org.junit.Test
import java.nio.ByteBuffer

class GeometricVisionPipelineTest {

    private fun makeBoxFrame(width: Int, height: Int, boxX: Int, boxY: Int, seq: Long): CameraFrame {
        val buf = ByteBuffer.allocate(width * height)
        for (y in 0 until height) {
            for (x in 0 until width) {
                val inBox = (x in boxX until (boxX + 25)) && (y in boxY until (boxY + 25))
                buf.put(if (inBox) 240.toByte() else 20.toByte())
            }
        }
        buf.flip()
        return CameraFrame(
            sequenceNumber = seq,
            timestampNs = seq * 100_000_000L, // 100ms apart (10 FPS)
            width = width,
            height = height,
            yBuffer = buf,
            yRowStride = width,
            yPixelStride = 1,
        )
    }

    @Test
    fun pipeline_firstFrame_initializesAndEmitsCleanOutput() {
        val pipeline = GeometricVisionPipeline()
        val frame1 = makeBoxFrame(width = 120, height = 120, boxX = 40, boxY = 40, seq = 1L)

        val out1 = pipeline.processFrame(frame1)
        assertEquals(1L, out1.frameSequence)
        assertTrue("First frame output has no obstacles yet", out1.obstacles.isEmpty())
        assertTrue("Processing time is non-negative", out1.processingTimeMs >= 0L)
    }

    @Test
    fun pipeline_secondFrame_tracksAndProcessesDiagnosticCallback() {
        val pipeline = GeometricVisionPipeline()
        var callbackInvoked = false
        var diagnosticResult: GeometricVisionResult? = null
        pipeline.onGeometricResult = { res ->
            callbackInvoked = true
            diagnosticResult = res
        }

        val frame1 = makeBoxFrame(width = 120, height = 120, boxX = 40, boxY = 40, seq = 1L)
        val frame2 = makeBoxFrame(width = 120, height = 120, boxX = 42, boxY = 41, seq = 2L)

        pipeline.processFrame(frame1)
        val out2 = pipeline.processFrame(frame2)

        assertEquals(2L, out2.frameSequence)
        assertTrue("Callback should be invoked on tracked frame", callbackInvoked)
        assertNotNull("Diagnostic result must be populated", diagnosticResult)
        assertTrue("Should have processed some features", diagnosticResult!!.detectedFeaturesCount > 0)
    }

    @Test
    fun pipeline_reset_clearsTemporalHistory() {
        val pipeline = GeometricVisionPipeline()
        val frame1 = makeBoxFrame(width = 120, height = 120, boxX = 40, boxY = 40, seq = 1L)

        pipeline.processFrame(frame1)
        pipeline.reset()

        // After reset, the next frame is treated as a first frame
        val frame2 = makeBoxFrame(width = 120, height = 120, boxX = 42, boxY = 41, seq = 2L)
        val out2 = pipeline.processFrame(frame2)

        assertEquals(2L, out2.frameSequence)
        assertTrue("Post-reset frame has no tracked obstacles", out2.obstacles.isEmpty())
    }

    @Test
    fun pipeline_withImu_evaluatesVisualInertialState() {
        val pipeline = GeometricVisionPipeline()
        var diagnosticResult: GeometricVisionResult? = null
        pipeline.onGeometricResult = { res ->
            diagnosticResult = res
        }

        val frame1 = makeBoxFrame(width = 120, height = 120, boxX = 40, boxY = 40, seq = 1L)
        val frame2 = makeBoxFrame(width = 120, height = 120, boxX = 42, boxY = 40, seq = 2L)
        val imu = com.minesafety.roboeye.core.ImuReading(
            gyroZ = Math.toRadians(15.0).toFloat(),
            isAvailable = true,
            timestampNs = 200_000_000L,
        )

        pipeline.processFrame(frame1, imu)
        pipeline.processFrame(frame2, imu)

        assertNotNull(diagnosticResult)
        assertTrue("IMU should be reported available", diagnosticResult!!.isImuAvailable)
        assertNotNull(diagnosticResult!!.motionState)
        assertNotNull(diagnosticResult!!.consistency)
        assertTrue("Ransac iterations should be recorded", diagnosticResult!!.ransacIterations >= 0)
    }
}
