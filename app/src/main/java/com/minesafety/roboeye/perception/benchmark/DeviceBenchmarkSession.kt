package com.minesafety.roboeye.perception.benchmark

import android.content.Context
import com.minesafety.roboeye.perception.model.DetectorMetadata
import com.minesafety.roboeye.perception.model.DetectorModelId
import com.minesafety.roboeye.perception.model.ModelReadinessStatus
import com.minesafety.roboeye.perception.model.ModelRegistry
import java.util.UUID
import kotlin.math.roundToInt

/**
 * Deterministic benchmark session engine for on-device performance profiling (Phase 6E).
 *
 * Invariants:
 * 1. Warm-up samples are tracked separately and strictly excluded from steady-state statistics.
 * 2. Steady-state latency statistics (Min, P50, P95, Max, Mean) use exact deterministic formulas.
 * 3. Actual production inference rate Hz is calculated as completedFrames / elapsedSeconds,
 *    strictly separated from theoretical detector throughput (1000 / lastLatencyMs).
 * 4. All rates and percentiles are sanitized against NaN and Infinite values.
 * 5. Time injection allows fast, deterministic unit test verification.
 */
class DeviceBenchmarkSession(
    private val timeProvider: () -> Long = { System.currentTimeMillis() }
) {
    private val lock = Any()

    private var runId: String = ""
    private var startTimestampMs: Long = 0L
    private var modelId: DetectorModelId = DetectorModelId.YOLO11N
    private var selectedModelId: DetectorModelId = DetectorModelId.YOLO11N
    private var activeModelId: DetectorModelId? = null
    private var status: BenchmarkStatus = BenchmarkStatus.IDLE
    private var failureReason: String? = null

    private var warmupTarget: Int = 10
    private var warmupCompleted: Int = 0
    private var benchmarkDurationMs: Long = 60_000L

    private var initializationMs: Long = 0L
    private var firstInferenceMs: Long = 0L
    private var steadyStateStartTimeMs: Long = 0L
    private var steadyStateEndTimeMs: Long = 0L

    // Latency buckets: strictly separate warm-up from steady-state
    private val warmupLatencies = mutableListOf<Long>()
    private val steadyStateLatencies = mutableListOf<Long>()
    private var lastLatencyMs: Long = 0L

    // Frame accounting (Steady-state)
    private var submittedFrames: Long = 0L
    private var inferenceStartedFrames: Long = 0L
    private var completedFrames: Long = 0L
    private var skippedFrames: Long = 0L
    private var pendingReplacements: Long = 0L
    private var staleResults: Long = 0L
    private var inferenceFailures: Long = 0L

    // Observational Detection Metrics
    private var detectionFrames: Long = 0L
    private var totalDetections: Long = 0L

    // Environment info
    private var cameraResolution: String = "1280x720"
    private var cameraTargetFps: Int = 30
    private var observedCameraFps: Float = 0.0f
    private var perceptionTargetHz: Float = 4.0f
    private var backendName: String = "CPU"
    private var confidenceThreshold: Float = 0.35f
    private var nmsThreshold: Float = 0.45f
    private var deviceManufacturer: String = "UNKNOWN"
    private var deviceModel: String = "UNKNOWN"
    private var androidVersion: String = "UNKNOWN"
    private var apiLevel: Int = 0
    private var appVersion: String = "1.0.0"

    val currentStatus: BenchmarkStatus get() = synchronized(lock) { status }
    val currentRunId: String get() = synchronized(lock) { runId }

    companion object {
        fun sanitize(value: Float): Float {
            return if (value.isNaN() || value.isInfinite()) 0.0f else value
        }

        fun generateRunId(): String {
            val ts = System.currentTimeMillis()
            val rand = UUID.randomUUID().toString().replace("-", "").take(6).uppercase()
            return "BENCH-$ts-$rand"
        }
    }

    /**
     * Initializes and starts a new benchmark session.
     */
    fun startSession(
        model: DetectorModelId,
        activeModel: DetectorModelId?,
        context: Context? = null,
        warmupFrames: Int = 10,
        durationMs: Long = 60_000L,
        camResolution: String = "1280x720",
        camTargetFps: Int = 30,
        obsCameraFps: Float = 30.0f,
        targetPerceptionHz: Float = 4.0f,
        backend: String = "CPU",
        confThresh: Float = 0.35f,
        nmsThresh: Float = 0.45f,
        initDurationMs: Long = 0L,
        customRunId: String? = null,
    ): DeviceBenchmarkReport {
        synchronized(lock) {
            runId = customRunId ?: generateRunId()
            startTimestampMs = timeProvider()
            modelId = model
            selectedModelId = model
            activeModelId = activeModel
            warmupTarget = warmupFrames.coerceAtLeast(1)
            warmupCompleted = 0
            benchmarkDurationMs = durationMs.coerceAtLeast(1000L)
            initializationMs = initDurationMs

            steadyStateStartTimeMs = 0L
            steadyStateEndTimeMs = 0L
            firstInferenceMs = 0L
            lastLatencyMs = 0L

            warmupLatencies.clear()
            steadyStateLatencies.clear()

            submittedFrames = 0L
            inferenceStartedFrames = 0L
            completedFrames = 0L
            skippedFrames = 0L
            pendingReplacements = 0L
            staleResults = 0L
            inferenceFailures = 0L

            detectionFrames = 0L
            totalDetections = 0L

            cameraResolution = camResolution
            cameraTargetFps = camTargetFps
            observedCameraFps = sanitize(obsCameraFps)
            perceptionTargetHz = sanitize(targetPerceptionHz)
            backendName = backend
            confidenceThreshold = confThresh
            nmsThreshold = nmsThresh

            deviceManufacturer = DeviceSystemMonitor.getDeviceManufacturer()
            deviceModel = DeviceSystemMonitor.getDeviceModel()
            androidVersion = DeviceSystemMonitor.getAndroidVersion()
            apiLevel = DeviceSystemMonitor.getApiLevel()
            appVersion = DeviceSystemMonitor.getAppVersion(context)

            // Failure handling: If model is not loaded
            if (activeModel == null) {
                if (model == DetectorModelId.YOLO26N) {
                    status = BenchmarkStatus.NOT_BENCHMARKED
                    failureReason = "ASSET_UNAVAILABLE"
                } else {
                    status = BenchmarkStatus.FAILED
                    failureReason = "INITIALIZATION_FAILED"
                }
                return buildReportLocked(context)
            }

            // Strict invariant: selectedModelId == activeModelId
            if (model != activeModel) {
                status = BenchmarkStatus.FAILED
                failureReason = "CROSS_MODEL_MISMATCH"
                return buildReportLocked(context)
            }

            status = BenchmarkStatus.WARMING_UP
            failureReason = null
            return buildReportLocked(context)
        }
    }

    fun onFrameSubmitted() {
        synchronized(lock) {
            if (status == BenchmarkStatus.RUNNING_STEADY_STATE || status == BenchmarkStatus.WARMING_UP) {
                submittedFrames++
            }
        }
    }

    fun onInferenceStarted() {
        synchronized(lock) {
            if (status == BenchmarkStatus.RUNNING_STEADY_STATE || status == BenchmarkStatus.WARMING_UP) {
                inferenceStartedFrames++
            }
        }
    }

    fun onFrameSkipped() {
        synchronized(lock) {
            if (status == BenchmarkStatus.RUNNING_STEADY_STATE) {
                skippedFrames++
                pendingReplacements++
            }
        }
    }

    fun onStaleResultRejected() {
        synchronized(lock) {
            if (status == BenchmarkStatus.RUNNING_STEADY_STATE) {
                staleResults++
            }
        }
    }

    fun onInferenceSuccess(
        latencyMs: Long,
        detectedObjectsCount: Int,
        context: Context? = null,
    ): DeviceBenchmarkReport {
        synchronized(lock) {
            val now = timeProvider()
            val validLatency = latencyMs.coerceAtLeast(0L)
            lastLatencyMs = validLatency

            if (firstInferenceMs == 0L) {
                firstInferenceMs = validLatency
            }

            when (status) {
                BenchmarkStatus.WARMING_UP -> {
                    warmupLatencies.add(validLatency)
                    warmupCompleted++
                    if (warmupCompleted >= warmupTarget) {
                        // Warm-up completed! Transition to steady state
                        status = BenchmarkStatus.RUNNING_STEADY_STATE
                        steadyStateStartTimeMs = now
                    }
                }

                BenchmarkStatus.RUNNING_STEADY_STATE -> {
                    // Steady-state sample: recorded in steady-state metrics
                    completedFrames++
                    steadyStateLatencies.add(validLatency)

                    if (detectedObjectsCount > 0) {
                        detectionFrames++
                        totalDetections += detectedObjectsCount
                    }

                    val elapsedSteadyState = now - steadyStateStartTimeMs
                    if (elapsedSteadyState >= benchmarkDurationMs) {
                        // Completed steady-state duration
                        steadyStateEndTimeMs = now
                        status = BenchmarkStatus.COMPLETED
                    }
                }

                else -> {
                    // IDLE, COMPLETED, FAILED, CANCELLED
                }
            }

            return buildReportLocked(context)
        }
    }

    fun onInferenceFailure(
        errorReason: String = "RUNTIME_INFERENCE_FAILURE",
        context: Context? = null,
    ): DeviceBenchmarkReport {
        synchronized(lock) {
            inferenceFailures++
            if (status == BenchmarkStatus.WARMING_UP) {
                // Insufficient warm-up failure
                status = BenchmarkStatus.FAILED
                failureReason = "WARMUP_FAILED: $errorReason"
            }
            return buildReportLocked(context)
        }
    }

    fun cancel(reason: String = "USER_CANCELLED", context: Context? = null): DeviceBenchmarkReport {
        synchronized(lock) {
            if (status == BenchmarkStatus.WARMING_UP || status == BenchmarkStatus.RUNNING_STEADY_STATE) {
                status = BenchmarkStatus.CANCELLED
                failureReason = reason
            }
            return buildReportLocked(context)
        }
    }

    fun reset() {
        synchronized(lock) {
            status = BenchmarkStatus.IDLE
            failureReason = null
            warmupLatencies.clear()
            steadyStateLatencies.clear()
            warmupCompleted = 0
            submittedFrames = 0L
            inferenceStartedFrames = 0L
            completedFrames = 0L
            skippedFrames = 0L
            pendingReplacements = 0L
            staleResults = 0L
            inferenceFailures = 0L
            detectionFrames = 0L
            totalDetections = 0L
            firstInferenceMs = 0L
            lastLatencyMs = 0L
            steadyStateStartTimeMs = 0L
            steadyStateEndTimeMs = 0L
        }
    }

    fun currentReport(context: Context? = null): DeviceBenchmarkReport {
        synchronized(lock) {
            return buildReportLocked(context)
        }
    }

    private fun buildReportLocked(context: Context?): DeviceBenchmarkReport {
        val now = timeProvider()
        val metadata = ModelRegistry.getMetadata(modelId)

        val elapsedSteadyStateMs = when (status) {
            BenchmarkStatus.COMPLETED -> steadyStateEndTimeMs - steadyStateStartTimeMs
            BenchmarkStatus.RUNNING_STEADY_STATE -> now - steadyStateStartTimeMs
            else -> 0L
        }.coerceAtLeast(0L)

        // Latency calculations: STRICTLY on steadyStateLatencies (warm-up excluded)
        val sampleCount = steadyStateLatencies.size.toLong()
        val minLatency = if (steadyStateLatencies.isNotEmpty()) steadyStateLatencies.minOrNull() ?: 0L else 0L
        val maxLatency = if (steadyStateLatencies.isNotEmpty()) steadyStateLatencies.maxOrNull() ?: 0L else 0L
        val meanLatency = if (sampleCount > 0L) {
            sanitize((steadyStateLatencies.sum().toDouble() / sampleCount).toFloat())
        } else {
            0.0f
        }

        val p50 = computePercentile(steadyStateLatencies, 0.50f)
        val p95 = computePercentile(steadyStateLatencies, 0.95f)

        // Actual Production Rate Hz = completedFrames / (elapsedSteadyStateMs / 1000f)
        val productionRateHz = if (elapsedSteadyStateMs > 0L) {
            sanitize(completedFrames * 1000.0f / elapsedSteadyStateMs)
        } else {
            0.0f
        }

        // Theoretical Detector Throughput FPS = 1000f / lastLatencyMs
        val theoreticalThroughput = if (lastLatencyMs > 0L) {
            sanitize(1000.0f / lastLatencyMs)
        } else {
            0.0f
        }

        val avgDetections = if (completedFrames > 0L) {
            sanitize(totalDetections.toFloat() / completedFrames)
        } else {
            0.0f
        }

        val memoryUsage = DeviceSystemMonitor.getMemoryUsageMb()
        val memoryMax = DeviceSystemMonitor.getMemoryMaxMb()
        val thermal = DeviceSystemMonitor.getThermalStatus(context)

        return DeviceBenchmarkReport(
            runId = runId,
            timestampMs = startTimestampMs,
            deviceManufacturer = deviceManufacturer,
            deviceModel = deviceModel,
            androidVersion = androidVersion,
            apiLevel = apiLevel,
            appVersion = appVersion,
            modelId = modelId,
            selectedModelId = selectedModelId,
            activeModelId = activeModelId,
            modelReadiness = metadata.readinessStatus,
            modelInputResolution = "${metadata.inputWidth}x${metadata.inputHeight}",
            modelInputType = metadata.inputTensorType,
            backend = backendName,
            confidenceThreshold = confidenceThreshold,
            nmsThreshold = nmsThreshold,
            cameraResolution = cameraResolution,
            cameraTargetFps = cameraTargetFps,
            observedCameraFps = observedCameraFps,
            perceptionTargetHz = perceptionTargetHz,
            initializationMs = initializationMs,
            firstInferenceMs = firstInferenceMs,
            warmupTarget = warmupTarget,
            warmupCompleted = warmupCompleted,
            benchmarkDurationMs = benchmarkDurationMs,
            elapsedSteadyStateMs = elapsedSteadyStateMs,
            inferenceSamples = sampleCount,
            inferenceSuccesses = completedFrames,
            inferenceFailures = inferenceFailures,
            latencyMinMs = minLatency,
            latencyP50Ms = p50,
            latencyP95Ms = p95,
            latencyMaxMs = maxLatency,
            latencyMeanMs = meanLatency,
            submittedFrames = submittedFrames,
            inferenceStartedFrames = inferenceStartedFrames,
            completedFrames = completedFrames,
            skippedFrames = skippedFrames,
            pendingReplacements = pendingReplacements,
            staleResults = staleResults,
            productionInferenceRateHz = productionRateHz,
            theoreticalDetectorThroughputFps = theoreticalThroughput,
            detectionFrames = detectionFrames,
            totalDetections = totalDetections,
            averageDetectionsPerFrame = avgDetections,
            memoryUsageMb = sanitize(memoryUsage),
            memoryMaxMb = sanitize(memoryMax),
            thermalStatus = thermal,
            benchmarkStatus = status,
            failureReason = failureReason,
        )
    }

    private fun computePercentile(samples: List<Long>, percentile: Float): Float {
        if (samples.isEmpty()) return 0.0f
        val sorted = samples.sorted()
        val index = ((sorted.size - 1) * percentile).roundToInt().coerceIn(0, sorted.size - 1)
        return sanitize(sorted[index].toFloat())
    }
}
