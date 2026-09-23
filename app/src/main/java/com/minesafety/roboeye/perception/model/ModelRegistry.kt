package com.minesafety.roboeye.perception.model

import android.content.Context

/**
 * Central registry of supported neural object detection models for benchmark and runtime selection.
 */
object ModelRegistry {

    private val METADATA_YOLO11N = DetectorMetadata(
        modelId = DetectorModelId.YOLO11N,
        name = "YOLO11n LiteRT / TFLite",
        version = "11.0",
        family = "YOLO11",
        readinessStatus = ModelReadinessStatus.READY,
        assetFileName = DetectorModelId.YOLO11N.assetFileName,
        inputWidth = 640,
        inputHeight = 640,
        inputChannels = 3,
        inputTensorType = "Float32 RGB [0.0, 1.0]",
        outputFormat = "[1, 84, 8400] Float32 (cx, cy, w, h + 80 COCO classes)",
        notes = "Bundled verified model asset. Upright letterboxed input.",
    )

    /**
     * YOLO26n Metadata Specification.
     * Exported and verified from authentic yolo26n.pt checkpoint.
     * Tensor dimensions and layout verified via LiteRT interpreter.
     */
    private val METADATA_YOLO26N = DetectorMetadata(
        modelId = DetectorModelId.YOLO26N,
        name = "YOLO26n LiteRT / TFLite",
        version = "26.0",
        family = "YOLO26",
        readinessStatus = ModelReadinessStatus.READY,
        assetFileName = DetectorModelId.YOLO26N.assetFileName,
        inputWidth = 640,
        inputHeight = 640,
        inputChannels = 3,
        inputTensorType = "Float32 RGB [0.0, 1.0]",
        outputFormat = "[1, 84, 8400] Float32 (cx, cy, w, h + 80 COCO classes)",
        notes = "Bundled verified model asset exported from yolo26n.pt. Upright letterboxed input.",
    )

    private val METADATA_SSD = DetectorMetadata(
        modelId = DetectorModelId.SSD_MOBILENET,
        name = "SSD MobileNet V1 INT8",
        version = "1.0",
        family = "MobileNet",
        readinessStatus = ModelReadinessStatus.READY,
        assetFileName = DetectorModelId.SSD_MOBILENET.assetFileName,
        inputWidth = 300,
        inputHeight = 300,
        inputChannels = 3,
        inputTensorType = "Quantized UINT8",
        outputFormat = "TFLite Detection PostProcess (boxes, classes, scores, num_detections)",
        notes = "Bundled fallback detector.",
    )

    private val registry: Map<DetectorModelId, DetectorMetadata> = mapOf(
        DetectorModelId.YOLO11N to METADATA_YOLO11N,
        DetectorModelId.YOLO26N to METADATA_YOLO26N,
        DetectorModelId.SSD_MOBILENET to METADATA_SSD,
    )

    fun getMetadata(modelId: DetectorModelId): DetectorMetadata =
        registry[modelId] ?: DetectorMetadata(
            modelId = modelId,
            name = modelId.displayName,
            version = "unknown",
            family = "unknown",
            readinessStatus = ModelReadinessStatus.ERROR,
            assetFileName = modelId.assetFileName,
            inputWidth = 0,
            inputHeight = 0,
            inputChannels = 0,
            inputTensorType = "UNKNOWN",
            outputFormat = "UNKNOWN",
        )

    fun getAllModels(): List<DetectorMetadata> = registry.values.toList()

    fun isAssetAvailable(context: Context, modelId: DetectorModelId): Boolean {
        val meta = getMetadata(modelId)
        return try {
            context.assets.open(meta.assetFileName).close()
            true
        } catch (_: Exception) {
            false
        }
    }
}
