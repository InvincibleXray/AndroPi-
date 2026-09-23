package com.minesafety.roboeye.vision

import com.minesafety.roboeye.core.model.CameraFrame
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.min

/**
 * High-speed FAST (Features from Accelerated Segment Test) corner detector.
 *
 * Evaluates the 16-pixel Bresenham circle around candidates directly on the raw luminance Y-buffer:
 * - Ultra-lightweight CPU footprint suitable for MediaTek Helio A22 / budget cores
 * - 4-pixel cardinal pre-filter (pixels 1, 5, 9, 13) for instantaneous flat-patch rejection
 * - Continuous 9-pixel arc thresholding (FAST-9)
 * - Spatial grid bucketing for uniform field-of-view distribution
 */
class FastFeatureDetector(
    val threshold: Int = 25,
    val borderMargin: Int = 10,
    val minDistance: Int = 10,
    val sampleStep: Int = 2,
) : FeatureDetector {

    // Relative offsets (dx, dy) for the 16 circle pixels at radius 3
    private val circleOffsets = arrayOf(
        Pair(0, -3),  // 0 (1)
        Pair(1, -3),  // 1 (2)
        Pair(2, -2),  // 2 (3)
        Pair(3, -1),  // 3 (4)
        Pair(3, 0),   // 4 (5)
        Pair(3, 1),   // 5 (6)
        Pair(2, 2),   // 6 (7)
        Pair(1, 3),   // 7 (8)
        Pair(0, 3),   // 8 (9)
        Pair(-1, 3),  // 9 (10)
        Pair(-2, 2),  // 10 (11)
        Pair(-3, 1),  // 11 (12)
        Pair(-3, 0),  // 12 (13)
        Pair(-3, -1), // 13 (14)
        Pair(-2, -2), // 14 (15)
        Pair(-1, -3), // 15 (16)
    )

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

        val numCols = 8
        val numRows = 6
        val bucketWidth = (endX - startX) / numCols
        val bucketHeight = (endY - startY) / numRows
        val buckets = Array(numRows * numCols) { ArrayList<FeaturePoint>() }

        // Fast pixel byte accessor
        fun getP(px: Int, py: Int): Int =
            yBuf.get(py * rowStride + px * pixelStride).toInt() and 0xFF

        var y = startY
        while (y < endY) {
            var x = startX
            while (x < endX) {
                val ip = getP(x, y)
                val highT = ip + threshold
                val lowT = ip - threshold

                // Cardinal pre-filter: inspect pixels 0, 4, 8, 12 (1, 5, 9, 13)
                val p0 = getP(x + circleOffsets[0].first, y + circleOffsets[0].second)
                val p4 = getP(x + circleOffsets[4].first, y + circleOffsets[4].second)
                val p8 = getP(x + circleOffsets[8].first, y + circleOffsets[8].second)
                val p12 = getP(x + circleOffsets[12].first, y + circleOffsets[12].second)

                var brighterCount = 0
                var darkerCount = 0

                if (p0 > highT) brighterCount++ else if (p0 < lowT) darkerCount++
                if (p4 > highT) brighterCount++ else if (p4 < lowT) darkerCount++
                if (p8 > highT) brighterCount++ else if (p8 < lowT) darkerCount++
                if (p12 > highT) brighterCount++ else if (p12 < lowT) darkerCount++

                // Fast exit if fewer than 2 of the 4 cardinal pixels pass (FAST-9 theorem)
                if (brighterCount < 2 && darkerCount < 2) {
                    x += sampleStep
                    continue
                }

                // Full 16-pixel Bresenham circle check for 9 contiguous pixels
                val ring = IntArray(16)
                for (i in 0 until 16) {
                    ring[i] = getP(x + circleOffsets[i].first, y + circleOffsets[i].second)
                }

                var isCorner = false
                var score = 0.0f

                // Test brighter arc of length 9
                for (start in 0 until 16) {
                    var allBrighter = true
                    var diffSum = 0
                    for (k in 0 until 9) {
                        val valAt = ring[(start + k) % 16]
                        if (valAt <= highT) {
                            allBrighter = false
                            break
                        }
                        diffSum += abs(valAt - ip)
                    }
                    if (allBrighter) {
                        isCorner = true
                        score = diffSum.toFloat()
                        break
                    }
                }

                // Test darker arc of length 9 if not already marked
                if (!isCorner) {
                    for (start in 0 until 16) {
                        var allDarker = true
                        var diffSum = 0
                        for (k in 0 until 9) {
                            val valAt = ring[(start + k) % 16]
                            if (valAt >= lowT) {
                                allDarker = false
                                break
                            }
                            diffSum += abs(valAt - ip)
                        }
                        if (allDarker) {
                            isCorner = true
                            score = diffSum.toFloat()
                            break
                        }
                    }
                }

                if (isCorner) {
                    val pt = FeaturePoint(x.toFloat(), y.toFloat(), score)
                    val c = min(numCols - 1, max(0, (x - startX) / bucketWidth))
                    val rIdx = min(numRows - 1, max(0, (y - startY) / bucketHeight))
                    buckets[rIdx * numCols + c].add(pt)
                }

                x += sampleStep
            }
            y += sampleStep
        }

        val quotaPerBucket = max(1, (maxFeatures / (numCols * numRows)) + 1)
        val selected = ArrayList<FeaturePoint>(maxFeatures)

        for (b in buckets) {
            if (b.isEmpty()) continue
            b.sortByDescending { it.response }
            var added = 0
            for (cand in b) {
                var farEnough = true
                for (sel in selected) {
                    val dx = cand.x - sel.x
                    val dy = cand.y - sel.y
                    if (dx * dx + dy * dy < minDistance * minDistance) {
                        farEnough = false
                        break
                    }
                }
                if (farEnough) {
                    selected.add(cand)
                    added++
                    if (added >= quotaPerBucket) break
                }
            }
            if (selected.size >= maxFeatures) break
        }

        return selected.take(maxFeatures)
    }
}
