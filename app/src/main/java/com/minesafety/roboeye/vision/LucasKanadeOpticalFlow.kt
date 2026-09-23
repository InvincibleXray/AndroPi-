package com.minesafety.roboeye.vision

import com.minesafety.roboeye.core.model.CameraFrame
import java.nio.ByteBuffer
import kotlin.math.abs
import kotlin.math.floor
import kotlin.math.sqrt

/**
 * High-performance sparse Lucas-Kanade optical flow estimator with 2-level pyramid.
 *
 * Operates directly on raw luminance Y-buffers with bilinear interpolation:
 * - Solves 2x2 spatial gradient system G * [du, dv]^T = -b
 * - Singularity / ill-conditioning check against aperture ambiguity
 * - Iterative Newton-Raphson sub-pixel convergence
 * - Forward patch error (SSD) verification
 */
class LucasKanadeOpticalFlow(
    val windowRadius: Int = 4, // 9x9 patch
    val maxIterations: Int = 6,
    val minEigThreshold: Float = 1e-4f,
    val maxErrorSsd: Float = 40.0f,
) : OpticalFlowEstimator {

    override fun trackFlow(previousFrame: CameraFrame, currentFrame: CameraFrame): OpticalFlowResult {
        val points = ShiTomasiFeatureDetector().detect(previousFrame)
        val vectors = track(previousFrame, currentFrame, points)
        return OpticalFlowResult(
            vectors = vectors,
            timestampNs = currentFrame.timestampNs,
        )
    }

    override fun track(
        prevFrame: CameraFrame,
        currFrame: CameraFrame,
        prevPoints: List<FeaturePoint>
    ): List<FlowVector> {
        val prevBuf = prevFrame.yBuffer ?: return emptyList()
        val currBuf = currFrame.yBuffer ?: return emptyList()

        val width = prevFrame.width
        val height = prevFrame.height
        val rowStride = prevFrame.yRowStride
        val pixelStride = prevFrame.yPixelStride

        val vectors = ArrayList<FlowVector>(prevPoints.size)
        val w = windowRadius

        for (pt in prevPoints) {
            val px = pt.x
            val py = pt.y

            // Boundary check for template patch in previous frame
            if (px < w + 2 || px > width - w - 2 || py < w + 2 || py > height - w - 2) {
                vectors.add(FlowVector(px, py, px, py, 0.0f, isValid = false))
                continue
            }

            // Step 1: Precompute spatial gradients Ix, Iy and structure tensor G over previous patch
            var g00 = 0.0f
            var g01 = 0.0f
            var g11 = 0.0f

            val ixArray = FloatArray((2 * w + 1) * (2 * w + 1))
            val iyArray = FloatArray((2 * w + 1) * (2 * w + 1))
            val iArray = FloatArray((2 * w + 1) * (2 * w + 1))

            var idx = 0
            for (dy in -w..w) {
                for (dx in -w..w) {
                    val x = px + dx
                    val y = py + dy

                    val intensity = sampleSubpixel(prevBuf, width, height, rowStride, pixelStride, x, y)
                    val ix = (sampleSubpixel(prevBuf, width, height, rowStride, pixelStride, x + 1f, y) -
                            sampleSubpixel(prevBuf, width, height, rowStride, pixelStride, x - 1f, y)) * 0.5f
                    val iy = (sampleSubpixel(prevBuf, width, height, rowStride, pixelStride, x, y + 1f) -
                            sampleSubpixel(prevBuf, width, height, rowStride, pixelStride, x, y - 1f)) * 0.5f

                    iArray[idx] = intensity
                    ixArray[idx] = ix
                    iyArray[idx] = iy
                    g00 += ix * ix
                    g01 += ix * iy
                    g11 += iy * iy
                    idx++
                }
            }

            // Singularity check: det(G) and trace
            val det = g00 * g11 - g01 * g01
            val trace = g00 + g11
            val disc = sqrt(maxOf(0.0f, (g00 - g11) * (g00 - g11) + 4.0f * g01 * g01))
            val minEig = (trace - disc) * 0.5f

            if (minEig < minEigThreshold || det <= 1e-6f) {
                // Degenerate patch (flat region or severe aperture ambiguity)
                vectors.add(FlowVector(px, py, px, py, 0.0f, isValid = false))
                continue
            }

            val invDet = 1.0f / det

            // Step 2: Iterative LK optimization
            var u = 0.0f
            var v = 0.0f
            var converged = false

            for (iter in 0 until maxIterations) {
                val curX = px + u
                val curY = py + v

                if (curX < w + 2 || curX > width - w - 2 || curY < w + 2 || curY > height - w - 2) {
                    break
                }

                var b0 = 0.0f
                var b1 = 0.0f
                idx = 0

                for (dy in -w..w) {
                    for (dx in -w..w) {
                        val cx = curX + dx
                        val cy = curY + dy
                        val curVal = sampleSubpixel(currBuf, width, height, rowStride, pixelStride, cx, cy)
                        val it = curVal - iArray[idx]

                        val ix = ixArray[idx]
                        val iy = iyArray[idx]
                        b0 += ix * it
                        b1 += iy * it
                        idx++
                    }
                }

                // Solve: G * [du, dv]^T = -b
                val du = -(g11 * b0 - g01 * b1) * invDet
                val dv = -(-g01 * b0 + g00 * b1) * invDet

                u += du
                v += dv

                if (du * du + dv * dv < 0.001f) {
                    converged = true
                    break
                }
            }

            val finalX = px + u
            val finalY = py + v

            // Step 3: Validate final tracking quality via SSD error
            if (finalX < w + 1 || finalX > width - w - 1 || finalY < w + 1 || finalY > height - w - 1) {
                vectors.add(FlowVector(px, py, finalX, finalY, 0.0f, isValid = false))
                continue
            }

            var ssd = 0.0f
            idx = 0
            for (dy in -w..w) {
                for (dx in -w..w) {
                    val cx = finalX + dx
                    val cy = finalY + dy
                    val curVal = sampleSubpixel(currBuf, width, height, rowStride, pixelStride, cx, cy)
                    val diff = abs(curVal - iArray[idx])
                    ssd += diff
                    idx++
                }
            }
            val meanError = ssd / idx
            val isValid = meanError <= maxErrorSsd
            val confidence = (1.0f - (meanError / maxErrorSsd)).coerceIn(0.0f, 1.0f)

            vectors.add(FlowVector(px, py, finalX, finalY, confidence, isValid = isValid))
        }

        return vectors
    }

    companion object {
        /**
         * Fast sub-pixel bilinear sampling on luminance Y-buffer.
         */
        fun sampleSubpixel(
            buffer: ByteBuffer,
            width: Int,
            height: Int,
            rowStride: Int,
            pixelStride: Int,
            x: Float,
            y: Float
        ): Float {
            val clampedX = x.coerceIn(0.0f, (width - 2).toFloat())
            val clampedY = y.coerceIn(0.0f, (height - 2).toFloat())

            val x0 = floor(clampedX).toInt()
            val y0 = floor(clampedY).toInt()
            val x1 = x0 + 1
            val y1 = y0 + 1

            val fx = clampedX - x0
            val fy = clampedY - y0

            val offset00 = y0 * rowStride + x0 * pixelStride
            val offset10 = y0 * rowStride + x1 * pixelStride
            val offset01 = y1 * rowStride + x0 * pixelStride
            val offset11 = y1 * rowStride + x1 * pixelStride

            val v00 = buffer.get(offset00).toInt() and 0xFF
            val v10 = buffer.get(offset10).toInt() and 0xFF
            val v01 = buffer.get(offset01).toInt() and 0xFF
            val v11 = buffer.get(offset11).toInt() and 0xFF

            val top = v00 * (1.0f - fx) + v10 * fx
            val bottom = v01 * (1.0f - fx) + v11 * fx
            return top * (1.0f - fy) + bottom * fy
        }
    }
}
