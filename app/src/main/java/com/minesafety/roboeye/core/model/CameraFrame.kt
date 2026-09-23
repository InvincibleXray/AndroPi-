package com.minesafety.roboeye.core.model

import java.nio.ByteBuffer

/**
 * Decoupled camera frame domain model for the Rover Brain perception pipeline.
 *
 * Keeps image buffers, geometry, and timestamps independent of Android UI views
 * or CameraX [androidx.camera.core.ImageProxy] lifecycles.
 */
data class CameraFrame(
    val sequenceNumber: Long,
    val timestampNs: Long,
    val width: Int,
    val height: Int,
    val rotationDegrees: Int = 0,
    /** Primary luminance (Y) plane buffer. */
    val yBuffer: ByteBuffer? = null,
    val yRowStride: Int = width,
    val yPixelStride: Int = 1,
    /** Optional compressed JPEG payload for telemetry streaming or recording. */
    val jpegData: ByteArray? = null,
    /** Native pixel format (e.g. YUV_420_888, NV21, RGBA_8888). */
    val format: FrameFormat = FrameFormat.YUV_420_888,
) {
    enum class FrameFormat {
        YUV_420_888,
        NV21,
        RGBA_8888,
        JPEG_ONLY,
    }

    override fun equals(other: Any?): Boolean {
        if (this === other) return true
        if (javaClass != other?.javaClass) return false
        other as CameraFrame
        return sequenceNumber == other.sequenceNumber && timestampNs == other.timestampNs
    }

    override fun hashCode(): Int {
        var result = sequenceNumber.hashCode()
        result = 31 * result + timestampNs.hashCode()
        return result
    }
}
