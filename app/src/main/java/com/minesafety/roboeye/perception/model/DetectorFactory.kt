package com.minesafety.roboeye.perception.model

import android.content.Context
import android.util.Log
import com.minesafety.roboeye.perception.ObjectDetector
import com.minesafety.roboeye.perception.SsdMobileNetDetector
import com.minesafety.roboeye.perception.Yolo26Detector
import com.minesafety.roboeye.perception.YoloDetector

sealed interface DetectorCreationResult {
    val requestedModelId: DetectorModelId
    val activeModelId: DetectorModelId?
    val detector: ObjectDetector?
    val message: String

    data class Success(
        override val detector: ObjectDetector,
        override val requestedModelId: DetectorModelId,
        override val message: String = "Detector initialized successfully: ${detector.modelName}",
    ) : DetectorCreationResult {
        override val activeModelId: DetectorModelId get() = requestedModelId
    }

    data class Unavailable(
        override val requestedModelId: DetectorModelId,
        override val message: String,
        val status: ModelReadinessStatus = ModelReadinessStatus.ERROR,
    ) : DetectorCreationResult {
        override val detector: ObjectDetector? get() = null
        override val activeModelId: DetectorModelId? get() = null
    }
}

object DetectorFactory {
    private const val TAG = "DetectorFactory"

    // Optional provider override for unit testing without requiring native TFLite binaries
    internal var detectorProvider: ((DetectorModelId, Context) -> ObjectDetector?)? = null

    fun createDetector(
        modelId: DetectorModelId,
        context: Context,
    ): DetectorCreationResult {
        detectorProvider?.invoke(modelId, context)?.let { customDetector ->
            return if (customDetector.isLoaded) {
                DetectorCreationResult.Success(customDetector, modelId)
            } else {
                DetectorCreationResult.Unavailable(
                    requestedModelId = modelId,
                    message = "${modelId.displayName} failed to initialize.",
                    status = ModelReadinessStatus.ERROR,
                )
            }
        }

        val metadata = ModelRegistry.getMetadata(modelId)

        return when (modelId) {
            DetectorModelId.YOLO11N -> {
                try {
                    val yolo = YoloDetector(context)
                    if (yolo.isLoaded) {
                        DetectorCreationResult.Success(yolo, modelId)
                    } else {
                        Log.w(TAG, "YOLO11n model asset missing or failed to initialize. NO cross-model fallback will be activated.")
                        DetectorCreationResult.Unavailable(
                            requestedModelId = modelId,
                            message = "YOLO11n model asset missing or failed to initialize. AI perception unavailable.",
                            status = ModelReadinessStatus.ERROR,
                        )
                    }
                } catch (e: Exception) {
                    Log.e(TAG, "Exception initializing YOLO11n: ${e.message}. NO cross-model fallback will be activated.")
                    DetectorCreationResult.Unavailable(
                        requestedModelId = modelId,
                        message = "YOLO11n initialization failed: ${e.message}. AI perception unavailable.",
                        status = ModelReadinessStatus.ERROR,
                    )
                }
            }

            DetectorModelId.YOLO26N -> {
                try {
                    val yolo26 = Yolo26Detector(context)
                    if (yolo26.isLoaded) {
                        DetectorCreationResult.Success(yolo26, modelId)
                    } else {
                        val isMissing = !ModelRegistry.isAssetAvailable(context, modelId)
                        val status = if (isMissing) ModelReadinessStatus.PENDING_ASSET else ModelReadinessStatus.ERROR
                        val msg = if (isMissing) {
                            "YOLO26n model asset '${metadata.assetFileName}' is not bundled (${metadata.readinessStatus.description}). AI perception paused."
                        } else {
                            "YOLO26n initialization or tensor contract failed: ${yolo26.executionReport.fallbackReason ?: "Unknown error"}. AI perception paused."
                        }
                        Log.w(TAG, "$msg NO cross-model fallback will be activated.")
                        DetectorCreationResult.Unavailable(
                            requestedModelId = modelId,
                            message = msg,
                            status = status,
                        )
                    }
                } catch (e: Exception) {
                    Log.e(TAG, "Exception initializing YOLO26n: ${e.message}. NO cross-model fallback will be activated.")
                    DetectorCreationResult.Unavailable(
                        requestedModelId = modelId,
                        message = "YOLO26n initialization failed: ${e.message}. AI perception paused.",
                        status = ModelReadinessStatus.ERROR,
                    )
                }
            }

            DetectorModelId.SSD_MOBILENET -> {
                try {
                    val ssd = SsdMobileNetDetector(context)
                    if (ssd.isLoaded) {
                        DetectorCreationResult.Success(ssd, modelId)
                    } else {
                        Log.w(TAG, "SSD MobileNet V1 model asset missing or failed to initialize. NO cross-model fallback will be activated.")
                        DetectorCreationResult.Unavailable(
                            requestedModelId = modelId,
                            message = "SSD MobileNet V1 model asset missing or failed to initialize. AI perception unavailable.",
                            status = ModelReadinessStatus.ERROR,
                        )
                    }
                } catch (e: Exception) {
                    Log.e(TAG, "Exception initializing SSD MobileNet V1: ${e.message}")
                    DetectorCreationResult.Unavailable(
                        requestedModelId = modelId,
                        message = "SSD MobileNet V1 initialization failed: ${e.message}. AI perception unavailable.",
                        status = ModelReadinessStatus.ERROR,
                    )
                }
            }
        }
    }
}
