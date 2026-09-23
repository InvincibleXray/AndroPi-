package com.minesafety.roboeye.perception.benchmark

import com.minesafety.roboeye.perception.model.DetectorModelId
import com.minesafety.roboeye.perception.model.ModelReadinessStatus
import com.minesafety.roboeye.perception.model.ModelRegistry
import kotlin.math.roundToInt

/**
 * Thread-safe metrics collector for on-device detector benchmarking.
 *
 * Strictly separates:
 * 1. Inference Latency (hardware compute duration of the model itself)
 * 2. Detector Throughput (theoretical max FPS = 1000 / latency)
 * 3. Production Rate (actual pipeline execution frequency, throttled by CameraX / mailbox)
 *
 * Guarantees zero NaN or Infinite values through strict numeric sanitization.
 */
class BenchmarkMetrics(
    initialModelId: DetectorModelId = DetectorModelId.YOLO11N
) {
    private val lock = Any()

    @Volatile private var requestedModelId: DetectorModelId = initialModelId
    @Volatile private var activeModelId: DetectorModelId? = null
    @Volatile private var readinessStatus: ModelReadinessStatus = ModelReadinessStatus.READY
    @Volatile private var modelName: String = "None"

    private var initTimeMs: Long = 0L
    private var totalInferences: Long = 0L
    private var successfulInferences: Long = 0L
    private var failedInferences: Long = 0L
    private var skippedFrames: Long = 0L

    private var lastLatencyMs: Long = 0L
    private var sumLatencyMs: Double = 0.0
    private var minLatencyMs: Long = 0L
    private var maxLatencyMs: Long = 0L

    private val latencyHistory = ArrayDeque<Long>(LATENCY_WINDOW_SIZE)
    private val timestampHistory = ArrayDeque<Long>(TIMESTAMP_WINDOW_SIZE)
    private var lastTimestampMs: Long = 0L

    companion object {
        const val LATENCY_WINDOW_SIZE = 100
        const val TIMESTAMP_WINDOW_SIZE = 30

        fun sanitize(value: Float): Float {
            return if (value.isNaN() || value.isInfinite()) 0.0f else value
        }

        fun sanitize(value: Double): Double {
            return if (value.isNaN() || value.isInfinite()) 0.0 else value
        }
    }

    fun recordInitialization(
        durationMs: Long,
        activeModel: DetectorModelId?,
        name: String,
        readiness: ModelReadinessStatus = if (activeModel != null) ModelReadinessStatus.READY else ModelReadinessStatus.ERROR,
    ) {
        synchronized(lock) {
            initTimeMs = durationMs
            activeModelId = activeModel
            modelName = name
            readinessStatus = readiness
        }
    }

    // Overload for backwards compatibility
    fun recordInitialization(
        durationMs: Long,
        isFallback: Boolean,
        activeModel: DetectorModelId?,
        name: String,
        readiness: ModelReadinessStatus = if (activeModel != null) ModelReadinessStatus.READY else ModelReadinessStatus.ERROR,
    ) {
        recordInitialization(durationMs, activeModel, name, readiness)
    }

    fun recordSuccess(latencyMs: Long, timestampMs: Long = System.currentTimeMillis()) {
        synchronized(lock) {
            val validLatency = latencyMs.coerceAtLeast(0L)
            totalInferences++
            successfulInferences++
            lastLatencyMs = validLatency
            sumLatencyMs += validLatency
            lastTimestampMs = timestampMs

            if (successfulInferences == 1L) {
                minLatencyMs = validLatency
                maxLatencyMs = validLatency
            } else {
                if (validLatency < minLatencyMs) minLatencyMs = validLatency
                if (validLatency > maxLatencyMs) maxLatencyMs = validLatency
            }

            if (latencyHistory.size >= LATENCY_WINDOW_SIZE) {
                latencyHistory.removeFirst()
            }
            latencyHistory.addLast(validLatency)

            if (timestampHistory.size >= TIMESTAMP_WINDOW_SIZE) {
                timestampHistory.removeFirst()
            }
            timestampHistory.addLast(timestampMs)
        }
    }

    fun recordFailure(timestampMs: Long = System.currentTimeMillis()) {
        synchronized(lock) {
            totalInferences++
            failedInferences++
            lastTimestampMs = timestampMs
        }
    }

    fun recordSkippedFrame() {
        synchronized(lock) {
            skippedFrames++
        }
    }

    fun resetForModelSwitch(newModelId: DetectorModelId) {
        synchronized(lock) {
            requestedModelId = newModelId
            activeModelId = null
            readinessStatus = ModelRegistry.getMetadata(newModelId).readinessStatus
            modelName = "None"

            initTimeMs = 0L
            totalInferences = 0L
            successfulInferences = 0L
            failedInferences = 0L
            skippedFrames = 0L

            lastLatencyMs = 0L
            sumLatencyMs = 0.0
            minLatencyMs = 0L
            maxLatencyMs = 0L

            latencyHistory.clear()
            timestampHistory.clear()
            lastTimestampMs = 0L
        }
    }

    fun snapshot(): DetectorPerformanceSnapshot {
        synchronized(lock) {
            val avgLatency = if (successfulInferences > 0L) {
                sanitize((sumLatencyMs / successfulInferences).toFloat())
            } else {
                0.0f
            }

            val p50 = computePercentile(0.50f)
            val p95 = computePercentile(0.95f)

            // Detector Throughput (theoretical max FPS given inference latency)
            val throughputFps = if (lastLatencyMs > 0L) {
                sanitize(1000.0f / lastLatencyMs)
            } else {
                0.0f
            }

            // Production Rate (actual pipeline execution frequency)
            val productionHz = if (timestampHistory.size >= 2) {
                val durationMs = timestampHistory.last() - timestampHistory.first()
                if (durationMs > 0L) {
                    val rate = (timestampHistory.size - 1) * 1000.0f / durationMs
                    sanitize(rate)
                } else {
                    0.0f
                }
            } else {
                0.0f
            }

            return DetectorPerformanceSnapshot(
                requestedModelId = requestedModelId,
                activeModelId = activeModelId,
                readinessStatus = readinessStatus,
                isFallbackActive = false,
                modelName = modelName,
                totalInferences = totalInferences,
                successfulInferences = successfulInferences,
                failedInferences = failedInferences,
                skippedFrames = skippedFrames,
                initTimeMs = initTimeMs,
                lastLatencyMs = lastLatencyMs,
                avgLatencyMs = avgLatency,
                p50LatencyMs = p50,
                p95LatencyMs = p95,
                minLatencyMs = minLatencyMs,
                maxLatencyMs = maxLatencyMs,
                detectorThroughputFps = throughputFps,
                productionRateHz = productionHz,
                lastTimestampMs = lastTimestampMs,
            )
        }
    }

    private fun computePercentile(percentile: Float): Float {
        if (latencyHistory.isEmpty()) return 0.0f
        val sorted = latencyHistory.sorted()
        val index = ((sorted.size - 1) * percentile).roundToInt().coerceIn(0, sorted.size - 1)
        return sanitize(sorted[index].toFloat())
    }
}
