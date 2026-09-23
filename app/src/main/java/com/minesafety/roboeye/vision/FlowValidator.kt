package com.minesafety.roboeye.vision

import kotlin.math.abs

/**
 * Validates optical flow vectors and rejects outliers, extreme jumps, and noise.
 */
class FlowValidator(
    val maxJumpPx: Float = 45.0f,
    val minNoiseThresholdPx: Float = 0.3f,
    val boundaryMarginPx: Float = 6.0f,
) {

    data class ValidationResult(
        val validVectors: List<FlowVector>,
        val rejectedCount: Int,
        val stationaryCount: Int,
        val meanDisplacement: Float,
    )

    fun validate(
        vectors: List<FlowVector>,
        imageWidth: Int,
        imageHeight: Int
    ): ValidationResult {
        if (vectors.isEmpty()) {
            return ValidationResult(emptyList(), 0, 0, 0.0f)
        }

        var rejected = 0
        var stationary = 0
        val candidates = ArrayList<FlowVector>(vectors.size)

        for (vec in vectors) {
            if (!vec.isValid) {
                rejected++
                continue
            }

            // Boundary validation
            if (vec.currX < boundaryMarginPx || vec.currX > imageWidth - boundaryMarginPx ||
                vec.currY < boundaryMarginPx || vec.currY > imageHeight - boundaryMarginPx) {
                rejected++
                continue
            }

            val dx = abs(vec.dx)
            val dy = abs(vec.dy)
            val len = vec.length

            // Max jump rejection
            if (dx > maxJumpPx || dy > maxJumpPx || len > maxJumpPx) {
                rejected++
                continue
            }

            // Subpixel noise threshold
            if (len < minNoiseThresholdPx) {
                stationary++
                // Still considered valid but stationary
                candidates.add(vec.copy(currX = vec.prevX, currY = vec.prevY))
                continue
            }

            candidates.add(vec)
        }

        // Statistical outlier rejection using IQR on displacement lengths
        if (candidates.size >= 8) {
            val movingLengths = candidates.filter { it.length >= minNoiseThresholdPx }
                .map { it.length }
                .sorted()

            if (movingLengths.size >= 6) {
                val q1 = movingLengths[movingLengths.size / 4]
                val q3 = movingLengths[(movingLengths.size * 3) / 4]
                val iqr = q3 - q1
                val maxAllowed = q3 + 2.0f * iqr

                val filtered = ArrayList<FlowVector>(candidates.size)
                for (cand in candidates) {
                    if (cand.length >= minNoiseThresholdPx && cand.length > maxAllowed) {
                        rejected++
                    } else {
                        filtered.add(cand)
                    }
                }

                val sumDisp = filtered.fold(0.0f) { acc, v -> acc + v.length }
                val meanDisp = if (filtered.isNotEmpty()) sumDisp / filtered.size else 0.0f
                return ValidationResult(filtered, rejected, stationary, meanDisp)
            }
        }

        val sumDisp = candidates.fold(0.0f) { acc, v -> acc + v.length }
        val meanDisp = if (candidates.isNotEmpty()) sumDisp / candidates.size else 0.0f
        return ValidationResult(candidates, rejected, stationary, meanDisp)
    }
}
