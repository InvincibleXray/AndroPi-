package com.minesafety.roboeye.perception.model

/**
 * Model readiness status for benchmark tracking.
 */
enum class ModelReadinessStatus(val description: String) {
    READY("Ready for execution"),
    PENDING_ASSET("Blocked: Pending official model asset delivery"),
    FALLBACK_ACTIVE("Active via safe fallback detector"),
    ERROR("Model initialization or loading error"),
}

/**
 * Metadata descriptor for an on-device neural detector.
 */
data class DetectorMetadata(
    val modelId: DetectorModelId,
    val name: String,
    val version: String,
    val family: String,
    val readinessStatus: ModelReadinessStatus,
    val assetFileName: String,
    val inputWidth: Int,
    val inputHeight: Int,
    val inputChannels: Int,
    val inputTensorType: String,
    val outputFormat: String,
    val notes: String = "",
) {
    val isAvailable: Boolean get() = readinessStatus == ModelReadinessStatus.READY
}
