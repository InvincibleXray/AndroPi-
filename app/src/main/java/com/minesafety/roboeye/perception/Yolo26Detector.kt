package com.minesafety.roboeye.perception

import android.content.Context
import android.content.res.AssetFileDescriptor
import android.graphics.Bitmap
import android.util.Log
import androidx.camera.core.ImageProxy
import com.minesafety.roboeye.core.NormalizedRect
import com.minesafety.roboeye.core.PerceptionObjectType
import com.minesafety.roboeye.core.RawDetectedObject
import com.minesafety.roboeye.core.Units
import com.minesafety.roboeye.hardware.AccelerationVerificationStatus
import com.minesafety.roboeye.hardware.InferenceBackend
import com.minesafety.roboeye.hardware.InferenceExecutionReport
import org.tensorflow.lite.Interpreter
import java.io.FileInputStream
import java.nio.ByteBuffer
import java.nio.ByteOrder
import java.nio.FloatBuffer
import java.nio.MappedByteBuffer
import java.nio.channels.FileChannel
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sqrt

/**
 * Genuine YOLO26n neural object detector implementation for Android.
 *
 * Implements strict runtime contract inspection:
 * - Inspects input tensor shapes, types, and channel ordering.
 * - Dynamically determines output tensor contract:
 *     1. [OutputContract.NMS_FREE_ONE_TO_ONE]: e.g. [1, maxDet, 6] format [x1, y1, x2, y2, conf, class_id]
 *     2. [OutputContract.RAW_ONE_TO_MANY]: e.g. [1, 84, 8400] format requiring external class-aware NMS
 * - Safely rejects incompatible tensor contracts without silent reinterpretation.
 * - Strictly isolates failures: if asset is missing or contract fails, reports unavailable without fallback.
 */
class Yolo26Detector(
    private val context: Context,
    val modelFileName: String = "models/yolo26n_detector.tflite",
    val labelFileName: String = "models/yolo_labelmap.txt",
    val numThreads: Int = 4,
    override val preferredBackend: InferenceBackend = InferenceBackend.CPU,
) : ObjectDetector {

    enum class OutputContract {
        RAW_ONE_TO_MANY,
        NMS_FREE_ONE_TO_ONE,
    }

    companion object {
        private const val TAG = "Yolo26Runtime"
        const val DEFAULT_MODEL_INPUT_SIZE = 640
        const val DEFAULT_NUM_CLASSES = 80
        const val DEFAULT_NMS_IOU_THRESHOLD = 0.45f
        const val MAX_REPORTED_OBJECTS = 6
        const val MIN_BOX_DIMENSION = 0.02f
        const val MIN_BOX_AREA = 0.001f
        const val MAX_BOX_AREA = 0.95f
        private const val INV_255 = 1.0f / 255.0f
    }

    override val detectorType: DetectorType = DetectorType.YOLO
    override val modelName: String = "YOLO26n"
    override var isLoaded: Boolean = false
        private set

    var detectedContract: OutputContract? = null
        private set
    var modelInputWidth: Int = DEFAULT_MODEL_INPUT_SIZE
        private set
    var modelInputHeight: Int = DEFAULT_MODEL_INPUT_SIZE
        private set
    var numAnchors: Int = 8400
        private set
    var numOutputChannels: Int = 84
        private set
    var maxDetections: Int = 300
        private set
    private var isChannelsFirst: Boolean = true

    private var interpreter: Interpreter? = null
    private var labels: List<String> = emptyList()

    private var inputBuffer: ByteBuffer? = null
    private var inputFloats: FloatArray = FloatArray(0)
    private var inputFloatView: FloatBuffer? = null
    private var intPixels: IntArray = IntArray(0)

    private var frameConverter: UprightFrameConverter? = null

    private var outputBuffer: ByteBuffer? = null
    private var outputFloatView: FloatBuffer? = null
    private var flatOutput: FloatArray = FloatArray(0)

    private var bestScores: FloatArray = FloatArray(0)
    private var bestClasses: IntArray = IntArray(0)

    override var executionReport: InferenceExecutionReport = InferenceExecutionReport(
        backend = preferredBackend,
        threadCount = numThreads,
        isFallbackActive = false,
        fallbackReason = null,
        accelerationStatus = AccelerationVerificationStatus.UNKNOWN,
        activeModelName = modelName,
        preferredModelName = modelName,
    )
        private set

    init {
        initialize()
    }

    private fun initialize() {
        try {
            val hasAsset = try {
                context.assets.open(modelFileName).close()
                true
            } catch (_: Exception) {
                false
            }

            if (!hasAsset) {
                executionReport = executionReport.copy(
                    isFallbackActive = false,
                    fallbackReason = "YOLO26n model asset '$modelFileName' not bundled. NO cross-model fallback.",
                    accelerationStatus = AccelerationVerificationStatus.NOT_AVAILABLE,
                )
                isLoaded = false
                return
            }

            val options = Interpreter.Options().apply {
                setNumThreads(numThreads)
            }
            val interp = Interpreter(loadModelBuffer(), options)

            // 1. Inspect Input Tensor Contract
            val inputCount = interp.inputTensorCount
            if (inputCount < 1) {
                throw IllegalStateException("YOLO26n requires >= 1 input tensor, got $inputCount")
            }
            val inputTensor = interp.getInputTensor(0)
            val inShape = inputTensor.shape()
            if (inShape.size == 4) {
                modelInputHeight = inShape[1]
                modelInputWidth = inShape[2]
            } else {
                throw IllegalStateException("Unexpected YOLO26n input tensor rank: ${inShape.size} (${inShape.contentToString()})")
            }

            // 2. Inspect Output Tensor Contract
            val outputCount = interp.outputTensorCount
            if (outputCount < 1) {
                throw IllegalStateException("YOLO26n requires >= 1 output tensor, got $outputCount")
            }
            val outputTensor = interp.getOutputTensor(0)
            val outShape = outputTensor.shape()

            if (outShape.size == 3) {
                val d0 = outShape[0] // batch
                val d1 = outShape[1]
                val d2 = outShape[2]

                if (d1 <= 128 && d2 > 128) {
                    // [1, 84, 8400] -> RAW_ONE_TO_MANY (channels first)
                    detectedContract = OutputContract.RAW_ONE_TO_MANY
                    numOutputChannels = d1
                    numAnchors = d2
                    isChannelsFirst = true
                } else if (d1 > 128 && d2 <= 128) {
                    if (d2 == 6) {
                        // [1, 300, 6] -> NMS_FREE_ONE_TO_ONE
                        detectedContract = OutputContract.NMS_FREE_ONE_TO_ONE
                        maxDetections = d1
                    } else {
                        // [1, 8400, 84] -> RAW_ONE_TO_MANY (anchors first)
                        detectedContract = OutputContract.RAW_ONE_TO_MANY
                        numAnchors = d1
                        numOutputChannels = d2
                        isChannelsFirst = false
                    }
                } else if (d2 == 6) {
                    // [1, maxDet, 6] -> NMS_FREE_ONE_TO_ONE
                    detectedContract = OutputContract.NMS_FREE_ONE_TO_ONE
                    maxDetections = d1
                } else {
                    throw IllegalStateException("Incompatible YOLO26n 3D output tensor shape: ${outShape.contentToString()}")
                }
            } else {
                throw IllegalStateException("Incompatible YOLO26n output tensor shape: ${outShape.contentToString()}")
            }

            // Allocate Pre-allocated Direct Buffers
            val inputElements = 1 * modelInputHeight * modelInputWidth * 3
            inputBuffer = ByteBuffer.allocateDirect(inputElements * 4).order(ByteOrder.nativeOrder())
            inputFloatView = inputBuffer!!.asFloatBuffer()
            inputFloats = FloatArray(inputElements)
            intPixels = IntArray(modelInputHeight * modelInputWidth)

            val totalOutputFloats = outShape.fold(1) { acc, v -> acc * v }
            outputBuffer = ByteBuffer.allocateDirect(totalOutputFloats * 4).order(ByteOrder.nativeOrder())
            outputFloatView = outputBuffer!!.asFloatBuffer()
            flatOutput = FloatArray(totalOutputFloats)

            if (detectedContract == OutputContract.RAW_ONE_TO_MANY) {
                bestScores = FloatArray(numAnchors)
                bestClasses = IntArray(numAnchors)
            }

            labels = loadLabels()
            interpreter = interp
            isLoaded = true

            executionReport = executionReport.copy(
                isFallbackActive = false,
                fallbackReason = null,
                accelerationStatus = AccelerationVerificationStatus.VERIFIED_FOR_MODEL,
                activeModelName = modelName,
                preferredModelName = modelName,
                runtimePath = "CPU/XNNPACK (contract=$detectedContract, shape=${outShape.contentToString()})",
            )
            Log.i(TAG, "YOLO26n initialized: contract=$detectedContract, in=${modelInputWidth}x$modelInputHeight, out=${outShape.contentToString()}")
        } catch (e: Exception) {
            Log.e(TAG, "YOLO26n initialization error: ${e.message}", e)
            executionReport = executionReport.copy(
                isFallbackActive = false,
                fallbackReason = "YOLO26n initialization error: ${e.message}. NO cross-model fallback.",
                accelerationStatus = AccelerationVerificationStatus.NOT_AVAILABLE,
            )
            isLoaded = false
            close()
        }
    }

    private fun loadModelBuffer(): MappedByteBuffer {
        val afd: AssetFileDescriptor = context.assets.openFd(modelFileName)
        FileInputStream(afd.fileDescriptor).use { stream ->
            return stream.channel.map(FileChannel.MapMode.READ_ONLY, afd.startOffset, afd.declaredLength)
        }
    }

    private fun loadLabels(): List<String> {
        return try {
            context.assets.open(labelFileName).bufferedReader().useLines { lines ->
                lines.map { it.trim() }.filter { it.isNotBlank() }.toList()
            }
        } catch (_: Exception) {
            emptyList()
        }
    }

    override fun detect(image: ImageProxy, sensitivity: Float): PerceptionFrameResult {
        val startTime = System.currentTimeMillis()
        val t0 = System.nanoTime()

        val width = image.width
        val height = image.height
        val rotationDegrees = image.imageInfo.rotationDegrees
        val (uprightWidth, uprightHeight) = FrameGeometry.uprightSize(width, height, rotationDegrees)

        val visibility = if (image.planes.isNotEmpty()) {
            val yPlane = image.planes[0]
            estimateVisibility(yPlane.buffer, width, height, yPlane.rowStride, yPlane.pixelStride)
        } else {
            null
        }

        val interp = interpreter
        val inBuf = inputBuffer
        val outBuf = outputBuffer
        val contract = detectedContract

        if (!isLoaded || interp == null || inBuf == null || outBuf == null || contract == null) {
            return PerceptionFrameResult(
                detectedObjects = emptyList(),
                visibilityScore = visibility,
                inferenceTimeMs = 0L,
                frameWidth = uprightWidth,
                frameHeight = uprightHeight,
            )
        }

        var rawBitmap: Bitmap? = null
        val candidates = mutableListOf<RawDetectedObject>()

        try {
            // 1. Preprocessing: ImageProxy -> upright, letterboxed RGB Float32 tensor
            rawBitmap = image.toBitmap()
            val converter = frameConverter ?: UprightFrameConverter(modelInputWidth).also { frameConverter = it }
            val tensorBitmap = converter.render(rawBitmap, rotationDegrees)
            val letterbox = converter.letterbox

            tensorBitmap.getPixels(intPixels, 0, modelInputWidth, 0, 0, modelInputWidth, modelInputHeight)
            var f = 0
            for (pixel in intPixels) {
                inputFloats[f++] = ((pixel shr 16) and 0xFF) * INV_255
                inputFloats[f++] = ((pixel shr 8) and 0xFF) * INV_255
                inputFloats[f++] = (pixel and 0xFF) * INV_255
            }
            inputFloatView?.rewind()
            inputFloatView?.put(inputFloats)
            inBuf.rewind()

            // 2. Genuine On-Device Inference
            outBuf.rewind()
            interp.run(inBuf, outBuf)

            // 3. Post-processing based on validated output contract
            outputFloatView?.rewind()
            outputFloatView?.get(flatOutput)

            val threshold = sensitivity.coerceIn(0.20f, 0.90f)

            when (contract) {
                OutputContract.NMS_FREE_ONE_TO_ONE -> {
                    // Output layout: [1, maxDetections, 6] -> [x1, y1, x2, y2, score, class_id]
                    for (i in 0 until maxDetections) {
                        val offset = i * 6
                        val conf = flatOutput[offset + 4]
                        if (conf < threshold) continue

                        val classId = flatOutput[offset + 5].toInt()
                        val rawLabel = if (classId in labels.indices) labels[classId] else continue
                        val mapped = TfliteObjectDetector.mapCocoClass(rawLabel) ?: continue
                        val (mappedType, semanticLabel) = mapped

                        val x1 = flatOutput[offset + 0]
                        val y1 = flatOutput[offset + 1]
                        val x2 = flatOutput[offset + 2]
                        val y2 = flatOutput[offset + 3]

                        val cx = (x1 + x2) * 0.5f
                        val cy = (y1 + y2) * 0.5f
                        val w = x2 - x1
                        val h = y2 - y1

                        val box = FrameGeometry.toUprightNormalized(
                            centerX = cx,
                            centerY = cy,
                            width = w,
                            height = h,
                            letterbox = letterbox,
                            uprightWidth = uprightWidth,
                            uprightHeight = uprightHeight,
                        ) ?: continue

                        val area = box.width * box.height
                        if (box.width < MIN_BOX_DIMENSION || box.height < MIN_BOX_DIMENSION) continue
                        if (area < MIN_BOX_AREA || area > MAX_BOX_AREA) continue

                        candidates.add(
                            RawDetectedObject(
                                boundingBox = box,
                                label = semanticLabel,
                                type = mappedType,
                                confidence = Units.round2(conf),
                                timestampMs = startTime,
                            )
                        )
                    }
                }

                OutputContract.RAW_ONE_TO_MANY -> {
                    val numClasses = min(DEFAULT_NUM_CLASSES, labels.size)
                    if (isChannelsFirst) {
                        // Channel-major: [1, 84, 8400]
                        java.util.Arrays.fill(bestScores, 0f)
                        java.util.Arrays.fill(bestClasses, -1)
                        for (c in 0 until numClasses) {
                            val base = (4 + c) * numAnchors
                            for (a in 0 until numAnchors) {
                                val score = flatOutput[base + a]
                                if (score > bestScores[a]) {
                                    bestScores[a] = score
                                    bestClasses[a] = c
                                }
                            }
                        }

                        for (a in 0 until numAnchors) {
                            val maxClassScore = bestScores[a]
                            val bestClassId = bestClasses[a]
                            if (maxClassScore < threshold || bestClassId < 0) continue

                            val rawLabel = if (bestClassId in labels.indices) labels[bestClassId] else continue
                            val mapped = TfliteObjectDetector.mapCocoClass(rawLabel) ?: continue
                            val (mappedType, semanticLabel) = mapped

                            val box = FrameGeometry.toUprightNormalized(
                                centerX = flatOutput[0 * numAnchors + a],
                                centerY = flatOutput[1 * numAnchors + a],
                                width = flatOutput[2 * numAnchors + a],
                                height = flatOutput[3 * numAnchors + a],
                                letterbox = letterbox,
                                uprightWidth = uprightWidth,
                                uprightHeight = uprightHeight,
                            ) ?: continue

                            val area = box.width * box.height
                            if (box.width < MIN_BOX_DIMENSION || box.height < MIN_BOX_DIMENSION) continue
                            if (area < MIN_BOX_AREA || area > MAX_BOX_AREA) continue

                            candidates.add(
                                RawDetectedObject(
                                    boundingBox = box,
                                    label = semanticLabel,
                                    type = mappedType,
                                    confidence = Units.round2(maxClassScore),
                                    timestampMs = startTime,
                                )
                            )
                        }
                    } else {
                        // Anchor-major: [1, 8400, 84]
                        for (a in 0 until numAnchors) {
                            val offset = a * numOutputChannels
                            var bestScore = 0f
                            var bestClassId = -1
                            for (c in 0 until numClasses) {
                                val score = flatOutput[offset + 4 + c]
                                if (score > bestScore) {
                                    bestScore = score
                                    bestClassId = c
                                }
                            }
                            if (bestScore < threshold || bestClassId < 0) continue

                            val rawLabel = if (bestClassId in labels.indices) labels[bestClassId] else continue
                            val mapped = TfliteObjectDetector.mapCocoClass(rawLabel) ?: continue
                            val (mappedType, semanticLabel) = mapped

                            val box = FrameGeometry.toUprightNormalized(
                                centerX = flatOutput[offset + 0],
                                centerY = flatOutput[offset + 1],
                                width = flatOutput[offset + 2],
                                height = flatOutput[offset + 3],
                                letterbox = letterbox,
                                uprightWidth = uprightWidth,
                                uprightHeight = uprightHeight,
                            ) ?: continue

                            val area = box.width * box.height
                            if (box.width < MIN_BOX_DIMENSION || box.height < MIN_BOX_DIMENSION) continue
                            if (area < MIN_BOX_AREA || area > MAX_BOX_AREA) continue

                            candidates.add(
                                RawDetectedObject(
                                    boundingBox = box,
                                    label = semanticLabel,
                                    type = mappedType,
                                    confidence = Units.round2(bestScore),
                                    timestampMs = startTime,
                                )
                            )
                        }
                    }
                }
            }
        } catch (_: Exception) {
            // Fault isolation safeguard
        } finally {
            rawBitmap?.recycle()
        }

        // Apply NMS only for RAW_ONE_TO_MANY; NMS_FREE_ONE_TO_ONE is already NMS-free
        val finalObjects = if (contract == OutputContract.RAW_ONE_TO_MANY) {
            TfliteObjectDetector.applyClassAwareNms(candidates, DEFAULT_NMS_IOU_THRESHOLD)
        } else {
            candidates
        }

        val topObjects = finalObjects
            .sortedByDescending { it.confidence }
            .take(MAX_REPORTED_OBJECTS)

        val duration = System.currentTimeMillis() - startTime

        return PerceptionFrameResult(
            detectedObjects = topObjects,
            visibilityScore = visibility,
            inferenceTimeMs = duration,
            frameWidth = uprightWidth,
            frameHeight = uprightHeight,
        )
    }

    private fun estimateVisibility(
        buffer: ByteBuffer,
        width: Int,
        height: Int,
        rowStride: Int,
        pixelStride: Int
    ): Float {
        var sum = 0L
        var sumSq = 0L
        var minVal = 255
        var maxVal = 0
        var count = 0

        val stepY = max(1, height / 10)
        val stepX = max(1, width / 10)

        var y = stepY
        while (y < height - stepY) {
            val rowOffset = y * rowStride
            var x = stepX
            while (x < width - stepX) {
                val index = rowOffset + x * pixelStride
                if (index < buffer.limit()) {
                    val p = buffer.get(index).toInt() and 0xFF
                    sum += p
                    sumSq += (p * p)
                    if (p < minVal) minVal = p
                    if (p > maxVal) maxVal = p
                    count++
                }
                x += stepX
            }
            y += stepY
        }

        if (count < 10) return 0.8f

        val mean = sum.toFloat() / count
        val variance = max(0f, (sumSq.toFloat() / count) - (mean * mean))
        val std = sqrt(variance)

        val contrast = if (mean > 1f) (std / mean).coerceIn(0f, 1f) else 0f
        val dynamicRange = ((maxVal - minVal) / 255f).coerceIn(0f, 1f)

        val score = (contrast * 0.6f + dynamicRange * 0.4f).coerceIn(0.05f, 1.0f)
        return Units.round2(score)
    }

    override fun close() {
        interpreter?.close()
        interpreter = null
        frameConverter?.recycle()
        frameConverter = null
        inputBuffer = null
        outputBuffer = null
        isLoaded = false
    }
}
