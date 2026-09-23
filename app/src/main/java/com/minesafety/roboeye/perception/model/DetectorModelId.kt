package com.minesafety.roboeye.perception.model

/**
 * Model identifiers for on-device object detection benchmark and runtime selection.
 */
enum class DetectorModelId(
    val id: String,
    val displayName: String,
    val assetFileName: String,
    val isAssetBundled: Boolean,
) {
    YOLO11N(
        id = "yolo11n",
        displayName = "YOLO11n",
        assetFileName = "models/yolo_detector.tflite",
        isAssetBundled = true,
    ),
    YOLO26N(
        id = "yolo26n",
        displayName = "YOLO26n",
        assetFileName = "models/yolo26n_detector.tflite",
        isAssetBundled = false,
    ),
    SSD_MOBILENET(
        id = "ssd_mobilenet_v1",
        displayName = "SSD MobileNet V1 (INT8)",
        assetFileName = "models/ssd_mobilenet_v1.tflite",
        isAssetBundled = true,
    );

    val label: String get() = displayName

    companion object {
        fun fromId(id: String?): DetectorModelId =
            entries.firstOrNull { it.id.equals(id, ignoreCase = true) || it.name.equals(id, ignoreCase = true) }
                ?: YOLO11N
    }
}
