package com.minesafety.roboeye.perception

import androidx.camera.core.ImageProxy
import com.minesafety.roboeye.core.RawDetectedObject

/**
 * Per-stage wall-clock cost of one perception pass, in microseconds.
 */
data class PerceptionStageTimings(
    val bitmapUs: Long = 0L,
    val visibilityUs: Long = 0L,
    val renderUs: Long = 0L,
    val tensorFillUs: Long = 0L,
    val inferenceUs: Long = 0L,
    val outputCopyUs: Long = 0L,
    val decodeUs: Long = 0L,
    val nmsUs: Long = 0L,
    val totalUs: Long = 0L,
) {
    fun format(): String =
        "bitmap=${us(bitmapUs)} vis=${us(visibilityUs)} render=${us(renderUs)} " +
            "tensor=${us(tensorFillUs)} infer=${us(inferenceUs)} outcopy=${us(outputCopyUs)} " +
            "decode=${us(decodeUs)} nms=${us(nmsUs)} total=${us(totalUs)}"

    private fun us(value: Long): String = "${value / 1000}.${(value % 1000) / 100}ms"
}

/**
 * Result of a single frame perception pass.
 */
data class PerceptionFrameResult(
    val detectedObjects: List<RawDetectedObject>,
    val visibilityScore: Float?,
    val inferenceTimeMs: Long,
    val frameWidth: Int,
    val frameHeight: Int,
    val timings: PerceptionStageTimings? = null,
)

/**
 * Interface defining the perception engine.
 */
interface PerceptionEngine {
    fun processFrame(image: ImageProxy, sensitivity: Float): PerceptionFrameResult
}
