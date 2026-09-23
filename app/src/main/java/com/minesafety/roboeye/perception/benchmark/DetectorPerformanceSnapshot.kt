package com.minesafety.roboeye.perception.benchmark

import com.minesafety.roboeye.perception.model.DetectorModelId
import com.minesafety.roboeye.perception.model.ModelReadinessStatus

/**
 * Immutable snapshot of detector execution and benchmark performance.
 *
 * All rate and latency values are sanitized to prevent NaN or Infinite values.
 */
data class DetectorPerformanceSnapshot(
    val requestedModelId: DetectorModelId,
    val activeModelId: DetectorModelId?,
    val readinessStatus: ModelReadinessStatus,
    val isFallbackActive: Boolean = false,
    val modelName: String,
    val totalInferences: Long,
    val successfulInferences: Long,
    val failedInferences: Long,
    val skippedFrames: Long,
    val initTimeMs: Long,
    val lastLatencyMs: Long,
    val avgLatencyMs: Float,
    val p50LatencyMs: Float,
    val p95LatencyMs: Float,
    val minLatencyMs: Long,
    val maxLatencyMs: Long,
    val detectorThroughputFps: Float,
    val productionRateHz: Float,
    val lastTimestampMs: Long,
)
