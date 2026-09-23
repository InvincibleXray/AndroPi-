package com.minesafety.roboeye.perception

import android.content.Context
import androidx.camera.core.ImageProxy
import com.minesafety.roboeye.hardware.InferenceBackend
import com.minesafety.roboeye.hardware.InferenceExecutionReport

/**
 * Supported object detector backends.
 */
enum class DetectorType(val label: String) {
    SSD_MOBILENET("SSD MobileNet V1 (INT8)"),
    YOLO("YOLO Neural Detector"),
}

/**
 * Universal abstraction for neural object detectors.
 */
interface ObjectDetector : PerceptionEngine, AutoCloseable {
    val detectorType: DetectorType
    val modelName: String
    val isLoaded: Boolean
    val preferredBackend: InferenceBackend
    val executionReport: InferenceExecutionReport

    fun detect(image: ImageProxy, sensitivity: Float): PerceptionFrameResult

    override fun processFrame(image: ImageProxy, sensitivity: Float): PerceptionFrameResult =
        detect(image, sensitivity)
}

/**
 * Concrete implementation wrapping the verified SSD MobileNet V1 TFLite detector.
 */
class SsdMobileNetDetector(
    private val delegate: TfliteObjectDetector
) : ObjectDetector {

    constructor(
        context: Context,
        modelFileName: String = "models/ssd_mobilenet_v1.tflite",
        labelFileName: String = "models/labelmap.txt",
        numThreads: Int = 4,
        preferredBackend: InferenceBackend = InferenceBackend.CPU,
    ) : this(TfliteObjectDetector(context, modelFileName, labelFileName, numThreads, preferredBackend))

    override val detectorType: DetectorType = DetectorType.SSD_MOBILENET
    override val modelName: String = "SSD MobileNet V1 INT8 (Quantized UINT8)"

    /**
     * Whether the TFLite interpreter actually came up.
     *
     * This used to be a hardcoded `true`, which made the property a claim about the *intent* to
     * load a model rather than the outcome. [TfliteObjectDetector] swallows an initialization
     * failure and leaves its interpreter null, so a missing or corrupt `ssd_mobilenet_v1.tflite`
     * produced a detector that reported itself loaded and then returned zero detections for every
     * frame — indistinguishable, upstream, from a camera pointed at an empty haul road.
     * [AdaptiveObjectDetector] also routes on this flag, so the hardcoded value defeated its
     * fallback logic.
     */
    override val isLoaded: Boolean get() = delegate.isModelLoaded
    override val preferredBackend: InferenceBackend get() = delegate.preferredBackend
    override val executionReport: InferenceExecutionReport get() = delegate.executionReport.copy(
        activeModelName = modelName,
        preferredModelName = "SSD MobileNet V1 INT8",
    )

    override fun detect(image: ImageProxy, sensitivity: Float): PerceptionFrameResult {
        return delegate.processFrame(image, sensitivity)
    }

    override fun close() {
        delegate.close()
    }
}
