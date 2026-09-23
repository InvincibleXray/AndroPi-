package com.minesafety.roboeye.vision

import com.minesafety.roboeye.core.model.CameraFrame
import java.nio.ByteBuffer
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sqrt

/**
 * Interface for 2D visual feature detection on [CameraFrame].
 */
interface FeatureDetector {
    fun detect(frame: CameraFrame, maxFeatures: Int = 150): List<FeaturePoint>
}

/**
 * High-performance Shi-Tomasi (Good Features to Track) corner detector.
 *
 * Computes structure tensor eigenvalues directly on the raw luminance Y-buffer:
 * - Zero RGB / Bitmap conversion
 * - Minimum eigenvalue score: R = min(lambda1, lambda2)
 * - Spatial bucket / grid-based non-maximum suppression for uniform field-of-view coverage
 * - Border exclusion margin
 */
class ShiTomasiFeatureDetector(
    val minQuality: Float = 0.01f,
    val minDistance: Int = 10,
    val patchRadius: Int = 2,
    val borderMargin: Int = 12,
    val sampleStep: Int = 2,
) : FeatureDetector {

    override fun detect(frame: CameraFrame, maxFeatures: Int): List<FeaturePoint> {
        val yBuf = frame.yBuffer ?: return emptyList()
        val width = frame.width
        val height = frame.height
        val rowStride = frame.yRowStride
        val pixelStride = frame.yPixelStride

        if (width < borderMargin * 2 || height < borderMargin * 2) return emptyList()

        val startX = borderMargin
        val endX = width - borderMargin
        val startY = borderMargin
        val endY = height - borderMargin

        // Step 1: Scan candidates and find max eigenvalue response for relative thresholding
        var maxScore = 0.0f
        val candidates = ArrayList<FeaturePoint>(maxFeatures * 2)

        // Grid bucketing: 8 horizontal x 6 vertical buckets
        val numCols = 8
        val numRows = 6
        val bucketWidth = (endX - startX) / numCols
        val bucketHeight = (endY - startY) / numRows
        val buckets = Array(numRows * numCols) { ArrayList<FeaturePoint>() }

        val r = patchRadius

        var y = startY
        while (y < endY) {
            var x = startX
            while (x < endX) {
                // Compute 2x2 structure tensor M over patch
                var m00 = 0.0f
                var m01 = 0.0f
                var m11 = 0.0f

                for (dy in -r..r) {
                    val py = y + dy
                    val lineOffset = py * rowStride
                    for (dx in -r..r) {
                        val px = x + dx
                        val pOffset = lineOffset + px * pixelStride

                        // Central differences
                        val ix = ((yBuf.get(pOffset + pixelStride).toInt() and 0xFF) -
                                (yBuf.get(pOffset - pixelStride).toInt() and 0xFF)) * 0.5f
                        val iy = ((yBuf.get(pOffset + rowStride).toInt() and 0xFF) -
                                (yBuf.get(pOffset - rowStride).toInt() and 0xFF)) * 0.5f

                        m00 += ix * ix
                        m01 += ix * iy
                        m11 += iy * iy
                    }
                }

                // Minimum eigenvalue of [m00 m01; m01 m11]
                val trace = m00 + m11
                val diff = m00 - m11
                val disc = sqrt(diff * diff + 4.0f * m01 * m01)
                val lambdaMin = (trace - disc) * 0.5f

                if (lambdaMin > 10.0f) { // base noise floor
                    if (lambdaMin > maxScore) {
                        maxScore = lambdaMin
                    }
                    val pt = FeaturePoint(x.toFloat(), y.toFloat(), lambdaMin)
                    val c = min(numCols - 1, max(0, (x - startX) / bucketWidth))
                    val rIdx = min(numRows - 1, max(0, (y - startY) / bucketHeight))
                    buckets[rIdx * numCols + c].add(pt)
                }

                x += sampleStep
            }
            y += sampleStep
        }

        if (maxScore <= 0.0f) return emptyList()

        val scoreThreshold = maxScore * minQuality
        val quotaPerBucket = max(1, (maxFeatures / (numCols * numRows)) + 1)
        val selected = ArrayList<FeaturePoint>(maxFeatures)

        for (b in buckets) {
            if (b.isEmpty()) continue
            b.sortByDescending { it.response }
            var addedFromBucket = 0
            for (candidate in b) {
                if (candidate.response < scoreThreshold) break
                // Distance suppression against already selected points in bucket
                var isFarEnough = true
                for (sel in selected) {
                    val dX = candidate.x - sel.x
                    val dY = candidate.y - sel.y
                    if (dX * dX + dY * dY < minDistance * minDistance) {
                        isFarEnough = false
                        break
                    }
                }
                if (isFarEnough) {
                    selected.add(candidate)
                    addedFromBucket++
                    if (addedFromBucket >= quotaPerBucket) break
                }
            }
            if (selected.size >= maxFeatures) break
        }

        return selected.take(maxFeatures)
    }
}
