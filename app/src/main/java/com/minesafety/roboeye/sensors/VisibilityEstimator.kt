package com.minesafety.roboeye.sensors

import com.minesafety.roboeye.core.Units
import java.nio.ByteBuffer

/**
 * Output of the on-device visibility heuristic.
 *
 * This is a **deterministic image statistic**, not a machine-learning model. It is the
 * same arithmetic the backend runs in `Robo Web App/backend/app/ai/visibility.py`; the UI
 * labels it "Prototype visibility estimation" for exactly that reason. It correlates with
 * fog but also reacts to darkness, a low-texture wall, and a dirty lens.
 */
data class VisibilityResult(
  /** Blended score, 0..1. Higher = clearer. */
  val score: Float,
  /** `std / mean` — fog flattens this first. */
  val contrast: Float,
  /** Laplacian variance (3×3 aperture). */
  val sharpness: Float,
  /** `(max - min) / 255`. */
  val dynamicRange: Float,
  /** `mean / 255`, reported so darkness can be distinguished from fog. */
  val brightness: Float,
  val width: Int,
  val height: Int,
  val timestamp: Long,
) {
  /** Bands mirror `SafetyThresholds.visibility_*` in `backend/app/core/config.py`. */
  val band: VisibilityBand
    get() =
      when {
        score < 0.10f -> VisibilityBand.STOP
        score < 0.15f -> VisibilityBand.VERY_LOW
        score < 0.35f -> VisibilityBand.LOW
        else -> VisibilityBand.CLEAR
      }

  val percent: Int get() = (score * 100f + 0.5f).toInt()
}

enum class VisibilityBand(val label: String) {
  CLEAR("CLEAR"),
  LOW("LOW"),
  VERY_LOW("VERY LOW"),
  STOP("STOP THRESHOLD"),
}

/**
 * Port of the backend's visibility heuristic, kept numerically aligned on purpose.
 *
 * `score = 0.5·contrast_n + 0.3·sharpness_n + 0.2·dynamic_range`, with
 * `contrast_n = min(contrast / 0.6, 1)` and `sharpness_n = min(laplacianVar / 500, 1)`.
 *
 * The `/500` divisor is the backend's **OpenCV** branch (`visibility.py:68`); the
 * numpy-only fallback divides by 300. The backend venv for this project has OpenCV 4.9
 * installed, so `/500` is the live path and the one replicated here.
 *
 * Expect the phone's number and the backend's number for the same instant to differ by a
 * few percent: the phone scores the raw Y plane while the backend scores the JPEG-encoded
 * copy that was uploaded, and JPEG quantisation slightly lowers Laplacian variance. Both
 * are shown in the UI rather than hidden, so the discrepancy is visible instead of
 * surprising.
 */
object VisibilityEstimator {

  private const val CONTRAST_SCALE = 0.6f
  private const val SHARPNESS_SCALE = 500.0f

  /**
   * Scores a CameraX `YUV_420_888` luminance plane.
   *
   * @param y the Y plane buffer (may be padded — [rowStride] / [pixelStride] handle it)
   */
  fun fromLuminance(
    y: ByteBuffer,
    width: Int,
    height: Int,
    rowStride: Int,
    pixelStride: Int,
    timestamp: Long,
  ): VisibilityResult? {
    if (width <= 2 || height <= 2) return null
    val gray = FloatArray(width * height)
    y.rewind()
    var out = 0
    for (row in 0 until height) {
      var idx = row * rowStride
      for (col in 0 until width) {
        if (idx >= y.limit()) return null
        // Y is unsigned 0..255; Kotlin Bytes are signed.
        gray[out++] = (y.get(idx).toInt() and 0xFF).toFloat()
        idx += pixelStride
      }
    }
    return fromGray(gray, width, height, timestamp)
  }

  /** Pure entry point — no Android types, fully unit-testable. */
  fun fromGray(gray: FloatArray, width: Int, height: Int, timestamp: Long): VisibilityResult? {
    val n = width * height
    if (n <= 0 || gray.size < n) return null

    var sum = 0.0
    var sumSq = 0.0
    var min = Float.MAX_VALUE
    var max = -Float.MAX_VALUE
    for (i in 0 until n) {
      val v = gray[i]
      sum += v
      sumSq += v.toDouble() * v
      if (v < min) min = v
      if (v > max) max = v
    }
    val mean = sum / n
    // Population standard deviation — matches numpy's ddof=0 default.
    val variance = (sumSq / n - mean * mean).coerceAtLeast(0.0)
    val std = Math.sqrt(variance)

    val contrast = (std / (mean + 1e-6)).toFloat()
    val sharp = laplacianVariance(gray, width, height)
    val dynRange = (max - min) / 255f

    val contrastN = minOf(contrast / CONTRAST_SCALE, 1f)
    val sharpN = minOf(sharp / SHARPNESS_SCALE, 1f)
    val score = Units.clamp(0.5f * contrastN + 0.3f * sharpN + 0.2f * dynRange)

    return VisibilityResult(
      score = score,
      contrast = contrast,
      sharpness = sharp,
      dynamicRange = dynRange,
      brightness = (mean / 255.0).toFloat(),
      width = width,
      height = height,
      timestamp = timestamp,
    )
  }

  /**
   * Variance of the 3×3 Laplacian response, replicating
   * `cv2.Laplacian(gray, CV_32F).var()`.
   *
   * Kernel `[[0,1,0],[1,-4,1],[0,1,0]]` (OpenCV's `ksize=1` aperture) with
   * `BORDER_REFLECT_101` edge handling and population variance over *all* pixels,
   * borders included — the same set numpy's `.var()` averages over.
   */
  fun laplacianVariance(gray: FloatArray, width: Int, height: Int): Float {
    if (width < 3 || height < 3) return 0f
    val n = width * height
    var sum = 0.0
    var sumSq = 0.0
    for (row in 0 until height) {
      val rowUp = reflect101(row - 1, height) * width
      val rowDown = reflect101(row + 1, height) * width
      val rowMid = row * width
      for (col in 0 until width) {
        val left = gray[rowMid + reflect101(col - 1, width)]
        val right = gray[rowMid + reflect101(col + 1, width)]
        val v = gray[rowUp + col] + gray[rowDown + col] + left + right - 4f * gray[rowMid + col]
        sum += v
        sumSq += v.toDouble() * v
      }
    }
    val mean = sum / n
    return (sumSq / n - mean * mean).coerceAtLeast(0.0).toFloat()
  }

  /** `BORDER_REFLECT_101`: index −1 maps to 1, index n maps to n−2. */
  private fun reflect101(i: Int, n: Int): Int =
    when {
      i < 0 -> -i
      i >= n -> 2 * n - i - 2
      else -> i
    }
}
