package com.minesafety.roboeye.vision

import kotlin.math.abs
import kotlin.math.min

/**
 * Focus of Expansion (FOE) estimation using robust weighted least-squares line intersection.
 */
class RobustFoeEstimator(
    val minMovingVectors: Int = 6,
    val minDivergenceRatio: Float = 0.65f,
    val minDisplacementPx: Float = 0.5f,
) {

    data class FoeResult(
        val x: Float,
        val y: Float,
        val isDivergent: Boolean,
        val confidence: Float,
        val inlierCount: Int = 0,
        val divergenceRatio: Float = 0.0f,
        val isConvergent: Boolean = false,
    )

    fun estimate(
        vectors: List<FlowVector>,
        imageWidth: Int,
        imageHeight: Int
    ): FoeResult {
        val moving = vectors.filter { it.isValid && it.length >= minDisplacementPx }
        if (moving.size < minMovingVectors) {
            return FoeResult(
                x = imageWidth * 0.5f,
                y = imageHeight * 0.5f,
                isDivergent = false,
                confidence = 0.0f,
                inlierCount = 0
            )
        }

        // Weighted least squares intersection of displacement lines:
        // dy * x - dx * y = dy * px - dx * py
        var ata00 = 0.0
        var ata01 = 0.0
        var ata11 = 0.0
        var atc0 = 0.0
        var atc1 = 0.0

        for (v in moving) {
            val a = v.dy.toDouble()
            val b = -v.dx.toDouble()
            val c = a * v.prevX - v.dx.toDouble() * v.prevY
            val weight = (v.confidence * min(v.length, 8.0f)).toDouble()

            ata00 += weight * a * a
            ata01 += weight * a * b
            ata11 += weight * b * b
            atc0 += weight * a * c
            atc1 += weight * b * c
        }

        val det = ata00 * ata11 - ata01 * ata01
        // If determinant is very small, flow lines are parallel (pure translation/rotation, no FOE)
        if (abs(det) < 1e-5) {
            return FoeResult(
                x = imageWidth * 0.5f,
                y = imageHeight * 0.5f,
                isDivergent = false,
                confidence = 0.0f,
                inlierCount = moving.size
            )
        }

        val foeX = ((ata11 * atc0 - ata01 * atc1) / det).toFloat()
        val foeY = ((-ata01 * atc0 + ata00 * atc1) / det).toFloat()

        // Check if FOE is within reasonable bounds (e.g. within 2x image bounds)
        if (foeX < -imageWidth || foeX > 2 * imageWidth ||
            foeY < -imageHeight || foeY > 2 * imageHeight) {
            return FoeResult(
                x = imageWidth * 0.5f,
                y = imageHeight * 0.5f,
                isDivergent = false,
                confidence = 0.0f,
                inlierCount = 0
            )
        }

        // Divergence verification: vectors must point away from FOE for forward vehicle motion
        var expandingCount = 0
        for (v in moving) {
            val radX = v.prevX - foeX
            val radY = v.prevY - foeY
            val dot = v.dx * radX + v.dy * radY
            if (dot > 0.0f) {
                expandingCount++
            }
        }

        val divergenceRatio = expandingCount.toFloat() / moving.size
        val isDivergent = divergenceRatio >= minDivergenceRatio
        val isConvergent = divergenceRatio <= (1.0f - minDivergenceRatio)
        val sampleFactor = (moving.size.toFloat() / (moving.size + 2)).coerceIn(0.0f, 1.0f)
        val confidence = if (isDivergent) {
            (divergenceRatio * sampleFactor).coerceIn(0.0f, 1.0f)
        } else if (isConvergent) {
            ((1.0f - divergenceRatio) * sampleFactor).coerceIn(0.0f, 1.0f)
        } else {
            0.0f
        }

        return FoeResult(
            x = foeX,
            y = foeY,
            isDivergent = isDivergent,
            confidence = confidence,
            inlierCount = expandingCount,
            divergenceRatio = divergenceRatio,
            isConvergent = isConvergent,
        )
    }
}
