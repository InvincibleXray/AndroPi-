package com.minesafety.roboeye.perception

import android.content.Context
import android.content.res.AssetFileDescriptor
import android.graphics.Bitmap
import androidx.camera.core.ImageProxy
import com.minesafety.roboeye.core.NormalizedRect
import com.minesafety.roboeye.core.PerceptionObjectType
import com.minesafety.roboeye.core.RawDetectedObject
import com.minesafety.roboeye.core.Units
import com.minesafety.roboeye.hardware.AccelerationVerificationStatus
import com.minesafety.roboeye.hardware.InferenceBackend
import com.minesafety.roboeye.hardware.InferenceExecutionReport
import org.tensorflow.lite.Delegate
import org.tensorflow.lite.Interpreter
import java.io.FileInputStream
import java.nio.ByteBuffer
import java.nio.ByteOrder
import java.nio.MappedByteBuffer
import java.nio.channels.FileChannel
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sqrt

/**
 * Genuine on-device semantic object detector powered by TensorFlow Lite.
 *
 * Executes a quantized single-shot multibox detector (SSD MobileNet V1).
 */
class TfliteObjectDetector(
    context: Context,
    modelFileName: String = "models/ssd_mobilenet_v1.tflite",
    labelFileName: String = "models/labelmap.txt",
    numThreads: Int = 4,
    val preferredBackend: InferenceBackend = InferenceBackend.CPU,
) : PerceptionEngine, AutoCloseable {

    companion object {
        const val MODEL_INPUT_SIZE = 300
        const val MAX_OUTPUT_DETECTIONS = 10
        const val MAX_REPORTED_OBJECTS = 6
        const val DEFAULT_NMS_IOU_THRESHOLD = 0.45f
        const val MIN_BOX_DIMENSION = 0.02f
        const val MIN_BOX_AREA = 0.001f
        const val MAX_BOX_AREA = 0.95f

        fun calculateIoU(a: NormalizedRect, b: NormalizedRect): Float {
            val interLeft = max(a.left, b.left)
            val interTop = max(a.top, b.top)
            val interRight = min(a.right, b.right)
            val interBottom = min(a.bottom, b.bottom)

            val interWidth = max(0.0f, interRight - interLeft)
            val interHeight = max(0.0f, interBottom - interTop)
            val interArea = interWidth * interHeight

            val areaA = a.width * a.height
            val areaB = b.width * b.height
            val unionArea = areaA + areaB - interArea

            return if (unionArea > 0.0001f) interArea / unionArea else 0.0f
        }

        /**
         * Class-Aware Non-Maximum Suppression (NMS).
         */
        fun applyClassAwareNms(
            detections: List<RawDetectedObject>,
            iouThreshold: Float = DEFAULT_NMS_IOU_THRESHOLD,
        ): List<RawDetectedObject> {
            if (detections.isEmpty()) return emptyList()

            val sorted = detections.sortedByDescending { it.confidence }
            val selected = mutableListOf<RawDetectedObject>()

            for (candidate in sorted) {
                var shouldSuppress = false
                for (accepted in selected) {
                    if (candidate.type == accepted.type) {
                        val iou = calculateIoU(candidate.boundingBox, accepted.boundingBox)
                        if (iou >= iouThreshold) {
                            shouldSuppress = true
                            break
                        }
                    }
                }
                if (!shouldSuppress) {
                    selected.add(candidate)
                }
            }
            return selected
        }

        /**
         * COCO-to-Domain Semantic Mapping.
         */
        fun mapCocoClass(rawLabel: String): Pair<PerceptionObjectType, String>? {
            val label = rawLabel.lowercase().trim()
            return when (label) {
                "person" -> PerceptionObjectType.PERSON to "person"
                "car" -> PerceptionObjectType.VEHICLE to "car"
                "truck" -> PerceptionObjectType.VEHICLE to "truck"
                "bus" -> PerceptionObjectType.VEHICLE to "bus"
                "motorcycle" -> PerceptionObjectType.VEHICLE to "motorcycle"
                "bicycle" -> PerceptionObjectType.VEHICLE to "bicycle"
                "traffic light" -> PerceptionObjectType.ROAD_OBSTRUCTION to "traffic light"
                "stop sign" -> PerceptionObjectType.ROAD_OBSTRUCTION to "stop sign"
                "fire hydrant" -> PerceptionObjectType.ROAD_OBSTRUCTION to "fire hydrant"
                "bench" -> PerceptionObjectType.ROAD_OBSTRUCTION to "bench"
                "chair" -> PerceptionObjectType.OBSTACLE to "chair"
                "couch" -> PerceptionObjectType.OBSTACLE to "couch"
                "bed" -> PerceptionObjectType.OBSTACLE to "bed"
                "dining table" -> PerceptionObjectType.OBSTACLE to "dining table"
                "backpack" -> PerceptionObjectType.OBSTACLE to "backpack"
                "suitcase" -> PerceptionObjectType.OBSTACLE to "suitcase"
                "bottle" -> PerceptionObjectType.OBSTACLE to "bottle"
                "cup" -> PerceptionObjectType.OBSTACLE to "cup"
                "potted plant" -> PerceptionObjectType.OBSTACLE to "plant"
                "refrigerator" -> PerceptionObjectType.LARGE_OBJECT to "refrigerator"
                "tv" -> PerceptionObjectType.LARGE_OBJECT to "monitor/tv"
                "laptop" -> PerceptionObjectType.LARGE_OBJECT to "laptop"
                "cell phone" -> PerceptionObjectType.LARGE_OBJECT to "cell phone"
                else -> null
            }
        }

        fun loadModelFile(context: Context, path: String): MappedByteBuffer {
            val fileDescriptor: AssetFileDescriptor = context.assets.openFd(path)
            val inputStream = FileInputStream(fileDescriptor.fileDescriptor)
            val fileChannel = inputStream.channel
            val startOffset = fileDescriptor.startOffset
            val declaredLength = fileDescriptor.declaredLength
            return fileChannel.map(FileChannel.MapMode.READ_ONLY, startOffset, declaredLength)
        }

        fun initializeInterpreterWithFallback(
            context: Context,
            modelFileName: String,
            preferredBackend: InferenceBackend,
            numThreads: Int,
        ): Triple<Interpreter?, InferenceExecutionReport, Delegate?> {
            var interp: Interpreter? = null
            var report = InferenceExecutionReport(
                backend = InferenceBackend.CPU,
                threadCount = numThreads,
                isFallbackActive = false,
                fallbackReason = null,
                accelerationStatus = AccelerationVerificationStatus.NOT_AVAILABLE
            )
            var delegate: Delegate? = null

            val modelBuffer: MappedByteBuffer? = try {
                loadModelFile(context, modelFileName)
            } catch (e: Throwable) {
                null
            }

            if (modelBuffer == null) {
                return Triple(
                    null,
                    report.copy(fallbackReason = "Failed to load model file $modelFileName"),
                    null
                )
            }

            when (preferredBackend) {
                InferenceBackend.NNAPI -> {
                    try {
                        val delegateClass = Class.forName("org.tensorflow.lite.nnapi.NnApiDelegate")
                        val nnDelegate = delegateClass.getDeclaredConstructor().newInstance() as Delegate
                        val options = Interpreter.Options().apply {
                            addDelegate(nnDelegate)
                        }
                        interp = Interpreter(modelBuffer, options)
                        delegate = nnDelegate
                        report = InferenceExecutionReport(
                            backend = InferenceBackend.NNAPI,
                            threadCount = 1,
                            isFallbackActive = false,
                            fallbackReason = null,
                            accelerationStatus = AccelerationVerificationStatus.VERIFIED_FOR_MODEL
                        )
                    } catch (e: Throwable) {
                        val reason = "NNAPI initialization failed (${e.message ?: e.javaClass.simpleName}); fell back to CPU"
                        val cpuOptions = Interpreter.Options().apply {
                            setNumThreads(numThreads)
                        }
                        interp = try {
                            Interpreter(modelBuffer, cpuOptions)
                        } catch (t: Throwable) {
                            null
                        }
                        report = InferenceExecutionReport(
                            backend = InferenceBackend.CPU,
                            threadCount = numThreads,
                            isFallbackActive = true,
                            fallbackReason = reason,
                            accelerationStatus = AccelerationVerificationStatus.AVAILABLE_BUT_NOT_VERIFIED_FOR_MODEL
                        )
                    }
                }
                InferenceBackend.GPU -> {
                    try {
                        val delegateClass = Class.forName("org.tensorflow.lite.gpu.GpuDelegate")
                        val gpuDelegate = delegateClass.getDeclaredConstructor().newInstance() as Delegate
                        val options = Interpreter.Options().apply {
                            addDelegate(gpuDelegate)
                        }
                        interp = Interpreter(modelBuffer, options)
                        delegate = gpuDelegate
                        report = InferenceExecutionReport(
                            backend = InferenceBackend.GPU,
                            threadCount = 1,
                            isFallbackActive = false,
                            fallbackReason = null,
                            accelerationStatus = AccelerationVerificationStatus.VERIFIED_FOR_MODEL
                        )
                    } catch (e: Throwable) {
                        val reason = "GPU Delegate initialization failed (${e.message ?: e.javaClass.simpleName}); fell back to CPU"
                        val cpuOptions = Interpreter.Options().apply {
                            setNumThreads(numThreads)
                        }
                        interp = try {
                            Interpreter(modelBuffer, cpuOptions)
                        } catch (t: Throwable) {
                            null
                        }
                        report = InferenceExecutionReport(
                            backend = InferenceBackend.CPU,
                            threadCount = numThreads,
                            isFallbackActive = true,
                            fallbackReason = reason,
                            accelerationStatus = AccelerationVerificationStatus.AVAILABLE_BUT_NOT_VERIFIED_FOR_MODEL
                        )
                    }
                }
                InferenceBackend.CPU -> {
                    val cpuOptions = Interpreter.Options().apply {
                        setNumThreads(numThreads)
                    }
                    interp = try {
                        Interpreter(modelBuffer, cpuOptions)
                    } catch (t: Throwable) {
                        null
                    }
                    report = InferenceExecutionReport(
                        backend = InferenceBackend.CPU,
                        threadCount = numThreads,
                        isFallbackActive = false,
                        fallbackReason = null,
                        accelerationStatus = AccelerationVerificationStatus.NOT_AVAILABLE
                    )
                }
            }

            return Triple(interp, report, delegate)
        }
    }

    private var interpreter: Interpreter? = null
    private var activeDelegate: Delegate? = null
    val executionReport: InferenceExecutionReport
    private val labels: List<String>

    private val inputBuffer: ByteBuffer = ByteBuffer.allocateDirect(1 * MODEL_INPUT_SIZE * MODEL_INPUT_SIZE * 3).apply {
        order(ByteOrder.nativeOrder())
    }
    private val intPixels = IntArray(MODEL_INPUT_SIZE * MODEL_INPUT_SIZE)

    private val outputLocations = Array(1) { Array(MAX_OUTPUT_DETECTIONS) { FloatArray(4) } }
    private val outputClasses = Array(1) { FloatArray(MAX_OUTPUT_DETECTIONS) }
    private val outputScores = Array(1) { FloatArray(MAX_OUTPUT_DETECTIONS) }
    private val numDetections = FloatArray(1)

    private var frameConverter: UprightFrameConverter? = null

    private val outputMap = mapOf(
        0 to outputLocations,
        1 to outputClasses,
        2 to outputScores,
        3 to numDetections
    )

    init {
        labels = loadLabels(context, labelFileName)
        val (initInterp, initReport, delegate) = initializeInterpreterWithFallback(
            context = context,
            modelFileName = modelFileName,
            preferredBackend = preferredBackend,
            numThreads = numThreads
        )
        interpreter = initInterp
        executionReport = initReport
        activeDelegate = delegate
    }

    val isModelLoaded: Boolean get() = interpreter != null

    override fun processFrame(image: ImageProxy, sensitivity: Float): PerceptionFrameResult {
        val startTime = System.currentTimeMillis()
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
        if (interp == null) {
            val latency = System.currentTimeMillis() - startTime
            return PerceptionFrameResult(emptyList(), visibility, latency, uprightWidth, uprightHeight)
        }

        var rawBitmap: Bitmap? = null
        val rawDetectedObjects = mutableListOf<RawDetectedObject>()

        try {
            rawBitmap = image.toBitmap()
            val converter = frameConverter ?: UprightFrameConverter(MODEL_INPUT_SIZE).also { frameConverter = it }
            val tensorBitmap = converter.render(rawBitmap, rotationDegrees, stretchToFill = true)

            tensorBitmap.getPixels(intPixels, 0, MODEL_INPUT_SIZE, 0, 0, MODEL_INPUT_SIZE, MODEL_INPUT_SIZE)
            inputBuffer.rewind()
            for (pixel in intPixels) {
                val r = (pixel shr 16) and 0xFF
                val g = (pixel shr 8) and 0xFF
                val b = pixel and 0xFF
                inputBuffer.put(r.toByte())
                inputBuffer.put(g.toByte())
                inputBuffer.put(b.toByte())
            }

            interp.runForMultipleInputsOutputs(arrayOf(inputBuffer), outputMap)

            val count = min(MAX_OUTPUT_DETECTIONS, numDetections[0].toInt().coerceAtLeast(0))
            val threshold = sensitivity.coerceIn(0.20f, 0.90f)

            for (i in 0 until count) {
                val score = outputScores[0][i]
                if (score < threshold) continue

                val classId = outputClasses[0][i].toInt()
                val rawLabel = if (classId in labels.indices) labels[classId] else "unknown"

                val mapped = mapCocoClass(rawLabel) ?: continue
                val (mappedType, semanticLabel) = mapped

                val ymin = outputLocations[0][i][0].coerceIn(0f, 1f)
                val xmin = outputLocations[0][i][1].coerceIn(0f, 1f)
                val ymax = outputLocations[0][i][2].coerceIn(0f, 1f)
                val xmax = outputLocations[0][i][3].coerceIn(0f, 1f)

                if (xmax <= xmin || ymax <= ymin) continue
                val boxWidth = xmax - xmin
                val boxHeight = ymax - ymin
                val area = boxWidth * boxHeight
                if (boxWidth < MIN_BOX_DIMENSION || boxHeight < MIN_BOX_DIMENSION) continue
                if (area < MIN_BOX_AREA || area > MAX_BOX_AREA) continue

                val box = NormalizedRect(
                    left = xmin,
                    top = ymin,
                    right = xmax,
                    bottom = ymax
                )

                rawDetectedObjects.add(
                    RawDetectedObject(
                        boundingBox = box,
                        label = semanticLabel,
                        type = mappedType,
                        confidence = Units.round2(score),
                        timestampMs = startTime,
                    )
                )
            }
        } catch (_: Exception) {
        } finally {
            rawBitmap?.recycle()
        }

        val nmsFiltered = applyClassAwareNms(rawDetectedObjects, DEFAULT_NMS_IOU_THRESHOLD)
        val topObjects = nmsFiltered
            .sortedByDescending { it.confidence }
            .take(MAX_REPORTED_OBJECTS)

        val inferenceTime = System.currentTimeMillis() - startTime

        return PerceptionFrameResult(
            detectedObjects = topObjects,
            visibilityScore = visibility,
            inferenceTimeMs = inferenceTime,
            frameWidth = uprightWidth,
            frameHeight = uprightHeight,
        )
    }

    private fun loadLabels(context: Context, path: String): List<String> {
        return try {
            context.assets.open(path).bufferedReader().useLines { lines ->
                lines.map { it.trim() }.filter { it.isNotBlank() }.toList()
            }
        } catch (e: Exception) {
            emptyList()
        }
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
        (activeDelegate as? AutoCloseable)?.close()
        activeDelegate = null
        frameConverter?.recycle()
        frameConverter = null
    }
}
