package com.minesafety.roboeye.vision

import kotlin.math.sqrt

/**
 * Result of RANSAC geometric flow consensus filtering.
 */
data class RansacResult(
    val inliers: List<FlowVector>,
    val outliersCount: Int,
    val consensusDx: Float,
    val consensusDy: Float,
    val inlierRatio: Float,
    val iterationsRun: Int,
)

/**
 * Lightweight RANSAC geometric flow consensus filter.
 *
 * Rejects erratic feature tracks, independent object motion outliers, and tracking artifacts
 * by finding the dominant consensus 2D displacement field across the scene.
 *
 * Tailored for real-time operation on low-end hardware (e.g. Redmi 6A @ 10 FPS):
 * - Bounded iteration budget (default 18 iterations)
 * - Deterministic pseudo-random stride sampling
 * - Early exit if dominant consensus exceeds 85% inliers
 * - Minimal heap allocations
 */
class RansacFlowFilter(
    val maxIterations: Int = 18,
    val inlierThresholdPx: Float = 2.5f,
    val minInliers: Int = 6,
) {
    private val thresholdSq = inlierThresholdPx * inlierThresholdPx

    fun filter(vectors: List<FlowVector>): RansacResult {
        val n = vectors.size
        if (n < minInliers) {
            val meanDx = if (n > 0) vectors.map { it.dx }.average().toFloat() else 0.0f
            val meanDy = if (n > 0) vectors.map { it.dy }.average().toFloat() else 0.0f
            return RansacResult(
                inliers = vectors,
                outliersCount = 0,
                consensusDx = meanDx,
                consensusDy = meanDy,
                inlierRatio = if (n > 0) 1.0f else 0.0f,
                iterationsRun = 0,
            )
        }

        var bestInlierCount = 0
        var bestSampleDx = 0.0f
        var bestSampleDy = 0.0f
        var iterationsRun = 0

        // Deterministic linear-congruential stride to cover vector array uniformly without allocations
        val stride = if (n > 1) (n / 3).coerceAtLeast(1) else 1
        var sampleIdx = 0

        for (iter in 0 until maxIterations) {
            iterationsRun++
            val sample = vectors[sampleIdx % n]
            sampleIdx += stride + 7

            val sDx = sample.dx
            val sDy = sample.dy

            var currentInliers = 0
            for (i in 0 until n) {
                val v = vectors[i]
                val dDx = v.dx - sDx
                val dDy = v.dy - sDy
                if (dDx * dDx + dDy * dDy <= thresholdSq) {
                    currentInliers++
                }
            }

            if (currentInliers > bestInlierCount) {
                bestInlierCount = currentInliers
                bestSampleDx = sDx
                bestSampleDy = sDy

                // Early exit if overwhelming consensus is found
                if (bestInlierCount >= (n * 0.85f).toInt()) {
                    break
                }
            }
        }

        // Collect all inliers matching the best hypothesis
        val inliers = ArrayList<FlowVector>(bestInlierCount.coerceAtLeast(n / 2))
        var sumDx = 0.0f
        var sumDy = 0.0f

        for (i in 0 until n) {
            val v = vectors[i]
            val dDx = v.dx - bestSampleDx
            val dDy = v.dy - bestSampleDy
            if (dDx * dDx + dDy * dDy <= thresholdSq) {
                inliers.add(v)
                sumDx += v.dx
                sumDy += v.dy
            }
        }

        val refinedDx = if (inliers.isNotEmpty()) sumDx / inliers.size else bestSampleDx
        val refinedDy = if (inliers.isNotEmpty()) sumDy / inliers.size else bestSampleDy
        val inlierRatio = if (n > 0) inliers.size.toFloat() / n else 0.0f

        return RansacResult(
            inliers = inliers,
            outliersCount = n - inliers.size,
            consensusDx = refinedDx,
            consensusDy = refinedDy,
            inlierRatio = inlierRatio,
            iterationsRun = iterationsRun,
        )
    }
}
