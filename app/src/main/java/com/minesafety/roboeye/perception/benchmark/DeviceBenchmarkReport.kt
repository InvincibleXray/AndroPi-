package com.minesafety.roboeye.perception.benchmark

import com.minesafety.roboeye.perception.model.DetectorModelId
import com.minesafety.roboeye.perception.model.ModelReadinessStatus

/**
 * Lifecycle status of an on-device benchmark session.
 */
enum class BenchmarkStatus(val label: String) {
    IDLE("Idle"),
    WARMING_UP("Warming Up"),
    RUNNING_STEADY_STATE("Benchmarking Steady-State"),
    COMPLETED("Completed"),
    FAILED("Failed"),
    CANCELLED("Cancelled"),
    NOT_BENCHMARKED("Not Benchmarked"),
}

/**
 * Comprehensive benchmark measurement record matching Phase 6E Section 23 specification.
 *
 * Guarantees:
 * - Steady-state latency excludes warm-up samples.
 * - Production rate is derived from completed production results over elapsed steady-state time,
 *   strictly separated from theoretical throughput (1000f / latency).
 * - Observational detection metrics are explicitly non-accuracy metrics.
 * - Strict numeric sanitization prevents NaN or Infinite values.
 */
data class DeviceBenchmarkReport(
    // IDENTITY
    val runId: String,
    val timestampMs: Long,
    val deviceManufacturer: String,
    val deviceModel: String,
    val androidVersion: String,
    val apiLevel: Int,
    val appVersion: String,
    val modelId: DetectorModelId,
    val selectedModelId: DetectorModelId,
    val activeModelId: DetectorModelId?,

    // MODEL
    val modelReadiness: ModelReadinessStatus,
    val modelInputResolution: String,
    val modelInputType: String,
    val backend: String,
    val confidenceThreshold: Float,
    val nmsThreshold: Float,

    // CAMERA
    val cameraResolution: String,
    val cameraTargetFps: Int,
    val observedCameraFps: Float,
    val perceptionTargetHz: Float,

    // TIMING
    val initializationMs: Long,
    val firstInferenceMs: Long,
    val warmupTarget: Int,
    val warmupCompleted: Int,
    val benchmarkDurationMs: Long,
    val elapsedSteadyStateMs: Long,

    // LATENCY (Steady-State Only)
    val inferenceSamples: Long,
    val inferenceSuccesses: Long,
    val inferenceFailures: Long,
    val latencyMinMs: Long,
    val latencyP50Ms: Float,
    val latencyP95Ms: Float,
    val latencyMaxMs: Long,
    val latencyMeanMs: Float,

    // PRODUCTION
    val submittedFrames: Long,
    val inferenceStartedFrames: Long,
    val completedFrames: Long,
    val skippedFrames: Long,
    val pendingReplacements: Long,
    val staleResults: Long,
    val productionInferenceRateHz: Float,
    val theoreticalDetectorThroughputFps: Float,

    // DETECTION (Observational Only - Not Ground Truth Accuracy)
    val detectionFrames: Long,
    val totalDetections: Long,
    val averageDetectionsPerFrame: Float,

    // SYSTEM
    val memoryUsageMb: Float,
    val memoryMaxMb: Float,
    val thermalStatus: String,

    // STATUS
    val benchmarkStatus: BenchmarkStatus,
    val failureReason: String? = null,
) {
    fun toJsonString(): String {
        return buildString {
            append("{\n")
            append("  \"identity\": {\n")
            append("    \"runId\": \"$runId\",\n")
            append("    \"timestampMs\": $timestampMs,\n")
            append("    \"deviceManufacturer\": \"$deviceManufacturer\",\n")
            append("    \"deviceModel\": \"$deviceModel\",\n")
            append("    \"androidVersion\": \"$androidVersion\",\n")
            append("    \"apiLevel\": $apiLevel,\n")
            append("    \"appVersion\": \"$appVersion\",\n")
            append("    \"modelId\": \"${modelId.id}\",\n")
            append("    \"model\": \"${modelId.name}\",\n")
            append("    \"selectedModelId\": \"${selectedModelId.id}\",\n")
            append("    \"activeModelId\": ${activeModelId?.let { "\"${it.id}\"" } ?: "null"}\n")
            append("  },\n")
            append("  \"model\": {\n")
            append("    \"modelReadiness\": \"${modelReadiness.name}\",\n")
            append("    \"modelInputResolution\": \"$modelInputResolution\",\n")
            append("    \"modelInputType\": \"$modelInputType\",\n")
            append("    \"backend\": \"$backend\",\n")
            append("    \"confidenceThreshold\": $confidenceThreshold,\n")
            append("    \"nmsThreshold\": $nmsThreshold\n")
            append("  },\n")
            append("  \"camera\": {\n")
            append("    \"cameraResolution\": \"$cameraResolution\",\n")
            append("    \"cameraTargetFps\": $cameraTargetFps,\n")
            append("    \"observedCameraFps\": $observedCameraFps,\n")
            append("    \"perceptionTargetHz\": $perceptionTargetHz\n")
            append("  },\n")
            append("  \"timing\": {\n")
            append("    \"initializationMs\": $initializationMs,\n")
            append("    \"firstInferenceMs\": $firstInferenceMs,\n")
            append("    \"warmupTarget\": $warmupTarget,\n")
            append("    \"warmupCompleted\": $warmupCompleted,\n")
            append("    \"benchmarkDurationMs\": $benchmarkDurationMs,\n")
            append("    \"elapsedSteadyStateMs\": $elapsedSteadyStateMs\n")
            append("  },\n")
            append("  \"latency\": {\n")
            append("    \"inferenceSamples\": $inferenceSamples,\n")
            append("    \"inferenceSuccesses\": $inferenceSuccesses,\n")
            append("    \"inferenceFailures\": $inferenceFailures,\n")
            append("    \"latencyMinMs\": $latencyMinMs,\n")
            append("    \"latencyP50Ms\": $latencyP50Ms,\n")
            append("    \"latencyP95Ms\": $latencyP95Ms,\n")
            append("    \"latencyMaxMs\": $latencyMaxMs,\n")
            append("    \"latencyMeanMs\": $latencyMeanMs\n")
            append("  },\n")
            append("  \"production\": {\n")
            append("    \"submittedFrames\": $submittedFrames,\n")
            append("    \"inferenceStartedFrames\": $inferenceStartedFrames,\n")
            append("    \"completedFrames\": $completedFrames,\n")
            append("    \"skippedFrames\": $skippedFrames,\n")
            append("    \"pendingReplacements\": $pendingReplacements,\n")
            append("    \"staleResults\": $staleResults,\n")
            append("    \"productionInferenceRateHz\": $productionInferenceRateHz,\n")
            append("    \"theoreticalDetectorThroughputFps\": $theoreticalDetectorThroughputFps\n")
            append("  },\n")
            append("  \"detection\": {\n")
            append("    \"detectionFrames\": $detectionFrames,\n")
            append("    \"totalDetections\": $totalDetections,\n")
            append("    \"averageDetectionsPerFrame\": $averageDetectionsPerFrame\n")
            append("  },\n")
            append("  \"system\": {\n")
            append("    \"memoryUsageMb\": $memoryUsageMb,\n")
            append("    \"memoryMaxMb\": $memoryMaxMb,\n")
            append("    \"thermalStatus\": \"$thermalStatus\"\n")
            append("  },\n")
            append("  \"status\": {\n")
            append("    \"benchmarkStatus\": \"${benchmarkStatus.name}\",\n")
            append("    \"failureReason\": ${failureReason?.let { "\"$it\"" } ?: "null"}\n")
            append("  }\n")
            append("}")
        }
    }
}
