package com.minesafety.roboeye.perception

import android.content.Context
import android.content.res.AssetFileDescriptor
import android.graphics.Bitmap
import android.util.Log
import androidx.camera.core.ImageProxy
import com.minesafety.roboeye.core.PerceptionObjectType
import com.minesafety.roboeye.core.RawDetectedObject
import com.minesafety.roboeye.core.Units
import com.minesafety.roboeye.hardware.AccelerationVerificationStatus
import com.minesafety.roboeye.hardware.InferenceBackend
import com.minesafety.roboeye.hardware.InferenceExecutionReport
import org.tensorflow.lite.Interpreter
import org.tensorflow.lite.gpu.CompatibilityList
import org.tensorflow.lite.gpu.GpuDelegate
import java.io.FileInputStream
import java.nio.ByteBuffer
import java.nio.ByteOrder
import java.nio.FloatBuffer
import java.nio.MappedByteBuffer
import java.nio.channels.FileChannel
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sqrt

/**
 * Genuine YOLO11n neural object detector implementation for Android.
 *
 * Executes the official Ultralytics YOLO11n model exported to LiteRT/TFLite format.
 * Emits genuine, non-fabricated bounding boxes, class scores, and confidence values.
 *
 * Input Tensor:  [1, 640, 640, 3] Float32 RGB normalized to [0.0, 1.0].
 * Output Tensor: [1, 84, 8400] Float32 (cx, cy, w, h in 640px space + 80 COCO classes).
 *
 * The analyzer frame arrives in sensor orientation, so it is rotated upright by
 * `imageInfo.rotationDegrees` and letterboxed (aspect preserved) before inference --
 * see [FrameGeometry]. Boxes are mapped back to upright-frame normalized coordinates,
 * which is what [BearingEstimator] and [DistanceEstimator] expect.
 *
 * If the model asset is missing or fails to initialize, [isLoaded] safely returns false
 * and [AdaptiveObjectDetector] routes seamlessly to [SsdMobileNetDetector] without crashing.
 */
class YoloDetector(
    private val context: Context,
    val modelFileName: String = "models/yolo_detector.tflite",
    val labelFileName: String = "models/yolo_labelmap.txt",
    val numThreads: Int = 4,
    override val preferredBackend: InferenceBackend = InferenceBackend.CPU,
) : ObjectDetector {

    companion object {
        const val MODEL_INPUT_SIZE = 640
        const val NUM_ANCHORS = 8400
        const val NUM_CLASSES = 80
        const val NUM_OUTPUT_CHANNELS = 84 // 4 box coords + 80 class scores
        const val DEFAULT_NMS_IOU_THRESHOLD = 0.45f
        const val MAX_REPORTED_OBJECTS = 6
        const val MIN_BOX_DIMENSION = 0.02f
        const val MIN_BOX_AREA = 0.001f
        const val MAX_BOX_AREA = 0.95f

        /** Reciprocal of the 8-bit range, so normalization multiplies instead of divides. */
        private const val INV_255 = 1.0f / 255.0f

        private const val TAG = "YoloRuntime"

        /**
         * How much faster an alternative interpreter path must be before it is adopted.
         *
         * A margin rather than a strict comparison: two paths within a few percent are the
         * same path as far as the rover is concerned, and switching for noise would trade a
         * known-good configuration for nothing.
         */
        private const val REQUIRED_IMPROVEMENT = 0.90f

        /**
         * Largest per-element output difference tolerated when adopting an alternative path.
         *
         * The tensor carries class scores in [0, 1] alongside box coordinates in 0..640 pixel
         * space, so this bounds both to a fraction that cannot move a detection: box corners
         * shift by hundredths of a pixel, and only a score sitting within 0.02 of the
         * confidence threshold could change verdict. Measured drift between XNNPACK and the
         * built-in kernels on a float16 model was 0.0025, well inside this.
         */
        private const val MAX_OUTPUT_DRIFT = 0.02f

        /**
         * TFLite 2.16 applies the XNNPACK delegate to float models by default, so this is
         * what `Interpreter.Options` gives without asking.
         */
        private const val DEFAULT_PATH = "CPU/XNNPACK"

        /** XNNPACK explicitly disabled, leaving TFLite's own reference kernels. */
        private const val ALTERNATIVE_PATH = "CPU/builtin-kernels"

        /** Hardware-accelerated path, offered only where the vendor list vouches for it. */
        private const val GPU_PATH = "GPU-delegate"
    }

    override val detectorType: DetectorType = DetectorType.YOLO
    override val modelName: String = "YOLO11n"
    override var isLoaded: Boolean = false
        private set

    private var interpreter: Interpreter? = null
    private var labels: List<String> = emptyList()

    /**
     * Delegate backing [interpreter], when the calibrated path uses one. Held because a
     * delegate must outlive the interpreter that references it and be closed after it.
     */
    private var activeDelegate: AutoCloseable? = null

    /**
     * Guards the one-time execution-path measurement in [calibrateExecutionPath]. Set
     * before the measurement runs, so a failure cannot cause it to be retried per frame.
     */
    private var isPathCalibrated = false

    // Pre-allocated reusable input buffer: 1 x 640 x 640 x 3 x 4 bytes (Float32)
    private val inputBuffer: ByteBuffer = ByteBuffer.allocateDirect(1 * MODEL_INPUT_SIZE * MODEL_INPUT_SIZE * 3 * 4).apply {
        order(ByteOrder.nativeOrder())
    }

    /**
     * Staging array for the normalized tensor.
     *
     * Filling the direct buffer with 1.2M individual `putFloat` calls measured far slower
     * than filling a plain array and handing it over in one bulk copy, which lowers to a
     * single memory move. The view is cached because `asFloatBuffer()` allocates.
     */
    private val inputFloats = FloatArray(MODEL_INPUT_SIZE * MODEL_INPUT_SIZE * 3)
    private val inputFloatView: FloatBuffer = inputBuffer.asFloatBuffer()
    private val intPixels = IntArray(MODEL_INPUT_SIZE * MODEL_INPUT_SIZE)

    /**
     * Rotates the sensor frame upright and letterboxes it into the square input tensor.
     * Created on first frame so the detector can be constructed off-device.
     */
    private var frameConverter: UprightFrameConverter? = null

    // Pre-allocated reusable output buffer: 1 x 84 x 8400 x 4 bytes (Float32)
    private val outputBuffer: ByteBuffer = ByteBuffer.allocateDirect(1 * NUM_OUTPUT_CHANNELS * NUM_ANCHORS * 4).apply {
        order(ByteOrder.nativeOrder())
    }
    private val outputFloatView: FloatBuffer = outputBuffer.asFloatBuffer()
    private val flatOutput = FloatArray(NUM_OUTPUT_CHANNELS * NUM_ANCHORS)

    /**
     * Per-anchor class argmax, resolved one class row at a time.
     *
     * The output tensor is channel-major, so scanning `[class][anchor]` walks memory
     * sequentially, whereas the natural `[anchor][class]` nesting strides 8400 floats
     * (33 KB) per step and misses cache on almost every read. Same result, fewer stalls.
     */
    private val bestScores = FloatArray(NUM_ANCHORS)
    private val bestClasses = IntArray(NUM_ANCHORS)

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
                    isFallbackActive = true,
                    fallbackReason = "YOLO model asset '$modelFileName' not bundled; fallback to SSD MobileNet V1.",
                    accelerationStatus = AccelerationVerificationStatus.NOT_AVAILABLE,
                )
                isLoaded = false
                return
            }

            // Load model buffer into read-only mapped memory
            val options = Interpreter.Options().apply {
                setNumThreads(numThreads)
            }

            interpreter = Interpreter(loadModelBuffer(), options)
            labels = loadLabels()
            isLoaded = true

            executionReport = executionReport.copy(
                isFallbackActive = false,
                fallbackReason = null,
                accelerationStatus = AccelerationVerificationStatus.VERIFIED_FOR_MODEL,
                activeModelName = modelName,
                preferredModelName = modelName,
                runtimePath = "$DEFAULT_PATH (uncalibrated)",
            )
        } catch (e: Exception) {
            executionReport = executionReport.copy(
                isFallbackActive = true,
                fallbackReason = "YOLO initialization error: ${e.message}; falling back to SSD.",
                accelerationStatus = AccelerationVerificationStatus.NOT_AVAILABLE,
            )
            isLoaded = false
            interpreter?.close()
            interpreter = null
        }
    }

    private fun loadModelBuffer(): MappedByteBuffer {
        val afd: AssetFileDescriptor = context.assets.openFd(modelFileName)
        FileInputStream(afd.fileDescriptor).use { stream ->
            return stream.channel.map(FileChannel.MapMode.READ_ONLY, afd.startOffset, afd.declaredLength)
        }
    }

    /**
     * Measures the interpreter paths this device offers and keeps the fastest one that
     * produces the same numbers as the path already running.
     *
     * Why measured rather than configured: an isolated instrumented benchmark on the
     * 32-bit armeabi-v7a device available here ranked TFLite's built-in kernels 1.63x
     * *faster* than XNNPACK for this float16 model (838 ms vs 1369 ms at 4 threads), yet
     * the same comparison inside the running app reversed it (3593 ms vs 1818 ms). The
     * built-in kernels only win when they get all four cores -- 2509 ms at one thread
     * against 838 ms at four -- while XNNPACK measured flat across thread counts. A live
     * camera pipeline is competing for those cores, so a figure obtained on an idle device
     * predicts the wrong winner. Deciding it here, under the contention the rover actually
     * runs in, is the only measurement that answers the question.
     *
     * Correctness is part of the decision, not an afterthought. Float16 weights dequantize
     * slightly differently per kernel implementation, so each candidate's full output
     * tensor is compared element-wise against the incumbent's on the same real frame, and
     * a faster path that disagrees by more than [MAX_OUTPUT_DRIFT] is rejected.
     *
     * NNAPI is deliberately absent. It cannot be applied to this model at all -- the fp16
     * export leaves a MUL node with mismatched operand types, which the delegate rejects at
     * prepare time ("input1->type != input2->type (FLOAT32 != FLOAT16)"). That is a property
     * of the graph rather than of any device, and a failing delegate can spend tens of
     * seconds compiling before saying so, which is not worth paying on every launch.
     *
     * Runs once, on the perception worker thread, using the frame already staged in
     * [inputBuffer]. The first frame therefore takes several extra inference passes to
     * complete; every later frame runs on the winner.
     */
    private fun calibrateExecutionPath() {
        isPathCalibrated = true
        val incumbentInterpreter = interpreter ?: return

        // Separate scratch output so the shared [outputBuffer] is untouched.
        val scratch = ByteBuffer
            .allocateDirect(NUM_OUTPUT_CHANNELS * NUM_ANCHORS * 4)
            .order(ByteOrder.nativeOrder())

        val incumbent = RuntimeCandidate(DEFAULT_PATH, incumbentInterpreter, activeDelegate)
        val challengers = buildChallengers()
        val discardable = challengers.toMutableList()

        try {
            // Warm every path before timing any of them, then time them back to back.
            // Startup is a busy and rapidly changing CPU environment; warming A then timing
            // A then warming B then timing B would compare the two at different points in
            // that transient, whereas adjacent timed runs see roughly the same contention.
            val paths = listOf(incumbent) + challengers
            for (candidate in paths) candidate.warmUp(scratch)
            for (candidate in paths) candidate.timeOnePass(scratch)

            val incumbentOutput = incumbent.output
            if (incumbentOutput == null || incumbent.millis <= 0) {
                Log.w(TAG, "path calibration abandoned: $DEFAULT_PATH could not complete a pass")
                return
            }

            val verdicts = StringBuilder()
            var winner: RuntimeCandidate? = null
            for (candidate in challengers) {
                val output = candidate.output
                if (output == null || candidate.millis <= 0) {
                    verdicts.append(" ${candidate.name}=could-not-run")
                    continue
                }
                val drift = maxAbsDiff(incumbentOutput, output)
                val isFaster = candidate.millis < incumbent.millis * REQUIRED_IMPROVEMENT
                val isEquivalent = drift <= MAX_OUTPUT_DRIFT
                verdicts.append(
                    " ${candidate.name}=${candidate.millis}ms(drift=${"%.5f".format(drift)}," +
                        "faster=$isFaster,equivalent=$isEquivalent)"
                )
                if (isFaster && isEquivalent && candidate.millis < (winner?.millis ?: Long.MAX_VALUE)) {
                    winner = candidate
                }
            }

            if (winner != null) {
                interpreter = winner.interpreter
                activeDelegate = winner.delegate
                discardable.remove(winner)
                incumbent.close()
            }

            val chosen = winner ?: incumbent
            executionReport = executionReport.copy(runtimePath = "${chosen.name}/${numThreads}t")
            Log.i(
                TAG,
                "path calibration on ${android.os.Build.SUPPORTED_ABIS.firstOrNull()}: " +
                    "$DEFAULT_PATH=${incumbent.millis}ms$verdicts " +
                    "threads=$numThreads -> using ${chosen.name}",
            )
        } catch (e: Exception) {
            Log.w(TAG, "path calibration failed (${e.message}); keeping $DEFAULT_PATH")
        } finally {
            // Everything not adopted is released here, including on the failure path.
            for (candidate in discardable) candidate.close()
        }
    }

    /**
     * Builds the alternative interpreters to weigh against the one already running. A path
     * that cannot even be constructed is simply absent from the comparison.
     */
    private fun buildChallengers(): List<RuntimeCandidate> {
        val challengers = mutableListOf<RuntimeCandidate>()

        try {
            val interp = Interpreter(
                loadModelBuffer(),
                Interpreter.Options().apply {
                    setNumThreads(numThreads)
                    setUseXNNPACK(false)
                },
            )
            challengers += RuntimeCandidate(ALTERNATIVE_PATH, interp, null)
        } catch (e: Exception) {
            Log.i(TAG, "$ALTERNATIVE_PATH unavailable: ${e.message}")
        }

        // GPU is attempted only where the vendor compatibility list vouches for the driver.
        // Unsupported drivers can abort the process natively, which no catch block would
        // survive, so availability is checked rather than discovered by crashing.
        val compatibility = try {
            CompatibilityList()
        } catch (_: Throwable) {
            null
        }
        if (compatibility == null || !compatibility.isDelegateSupportedOnThisDevice) {
            Log.i(TAG, "$GPU_PATH not attempted: device is not on the GPU delegate compatibility list")
            return challengers
        }

        var delegate: GpuDelegate? = null
        try {
            delegate = GpuDelegate(compatibility.bestOptionsForThisDevice)
            val interp = Interpreter(
                loadModelBuffer(),
                Interpreter.Options().apply {
                    setNumThreads(numThreads)
                    addDelegate(delegate)
                },
            )
            challengers += RuntimeCandidate(GPU_PATH, interp, delegate)
            delegate = null // ownership transferred to the candidate
        } catch (e: Exception) {
            Log.i(TAG, "$GPU_PATH unavailable: ${e.message}")
        } finally {
            delegate?.close()
        }

        return challengers
    }

    /**
     * One interpreter configuration under evaluation, together with what it measured.
     *
     * The delegate outlives the interpreter that uses it and must be closed after it, which
     * is why the two travel together.
     */
    private inner class RuntimeCandidate(
        val name: String,
        val interpreter: Interpreter,
        val delegate: AutoCloseable?,
    ) {
        var millis: Long = -1
            private set
        var output: FloatArray? = null
            private set

        /**
         * Discarded pass: a fresh interpreter's first invocation includes lazy tensor
         * allocation and kernel setup, which is not what a rover frame pays.
         */
        fun warmUp(scratch: ByteBuffer) = runPass(scratch, capture = false)

        fun timeOnePass(scratch: ByteBuffer) = runPass(scratch, capture = true)

        private fun runPass(scratch: ByteBuffer, capture: Boolean) {
            try {
                inputBuffer.rewind()
                scratch.rewind()
                val startNs = System.nanoTime()
                interpreter.run(inputBuffer, scratch)
                if (!capture) return

                millis = (System.nanoTime() - startNs) / 1_000_000
                val captured = FloatArray(NUM_OUTPUT_CHANNELS * NUM_ANCHORS)
                scratch.rewind()
                scratch.asFloatBuffer().get(captured)
                output = captured
            } catch (e: Exception) {
                // A path that cannot execute the model is the normal outcome for a delegate
                // that fails to apply, not an error worth propagating.
                millis = -1
                output = null
                Log.i(TAG, "$name could not execute the model: ${e.message}")
            } finally {
                inputBuffer.rewind()
            }
        }

        fun close() {
            try {
                interpreter.close()
                delegate?.close()
            } catch (_: Exception) {
                // Releasing a candidate that already failed must not take the app down.
            }
        }
    }

    private fun maxAbsDiff(a: FloatArray, b: FloatArray): Float {
        var worst = 0f
        for (i in a.indices) {
            val diff = abs(a[i] - b[i])
            if (diff > worst) worst = diff
        }
        return worst
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
        var tVisibility = t0
        var tBitmap = t0
        var tRender = t0
        var tTensorFill = t0
        var tInference = t0
        var tOutputCopy = t0
        var tDecode = t0

        val width = image.width
        val height = image.height

        // The buffer is delivered in sensor orientation; rotationDegrees is the clockwise
        // rotation that makes it upright. The preview surface applies it, so the analyzer
        // must too, otherwise the model sees a 90-degree-rotated scene.
        val rotationDegrees = image.imageInfo.rotationDegrees
        val (uprightWidth, uprightHeight) = FrameGeometry.uprightSize(width, height, rotationDegrees)

        // Atmospheric visibility estimation from Y-plane
        // No luminance plane means no measurement: report null rather than the 1.0f ("perfectly
        // clear") this used to substitute. That value reached the backend's `visibility` field on a
        // packet marked simulated=false, where it holds down a safety rule.
        val visibility = if (image.planes.isNotEmpty()) {
            val yPlane = image.planes[0]
            estimateVisibility(yPlane.buffer, width, height, yPlane.rowStride, yPlane.pixelStride)
        } else {
            null
        }
        tVisibility = System.nanoTime()

        val interp = interpreter
        if (!isLoaded || interp == null) {
            return PerceptionFrameResult(
                detectedObjects = emptyList(),
                visibilityScore = visibility,
                inferenceTimeMs = 0L,
                frameWidth = uprightWidth,
                frameHeight = uprightHeight
            )
        }

        var rawBitmap: Bitmap? = null
        var isCalibrationFrame = false
        val candidates = mutableListOf<RawDetectedObject>()

        try {
            // 1. Preprocessing: ImageProxy -> upright, letterboxed 640x640 RGB Float32 tensor
            rawBitmap = image.toBitmap()
            tBitmap = System.nanoTime()

            val converter = frameConverter ?: UprightFrameConverter(MODEL_INPUT_SIZE).also { frameConverter = it }
            val tensorBitmap = converter.render(rawBitmap, rotationDegrees)
            val letterbox = converter.letterbox
            tRender = System.nanoTime()

            tensorBitmap.getPixels(intPixels, 0, MODEL_INPUT_SIZE, 0, 0, MODEL_INPUT_SIZE, MODEL_INPUT_SIZE)
            var f = 0
            for (pixel in intPixels) {
                inputFloats[f++] = ((pixel shr 16) and 0xFF) * INV_255
                inputFloats[f++] = ((pixel shr 8) and 0xFF) * INV_255
                inputFloats[f++] = (pixel and 0xFF) * INV_255
            }
            inputFloatView.rewind()
            inputFloatView.put(inputFloats)
            inputBuffer.rewind()
            tTensorFill = System.nanoTime()

            // One-time: pick the faster interpreter path using this real frame. Reported
            // via [PerceptionStageTimings] being absent, so the profiler does not average
            // this unrepresentative pass in with steady-state frames.
            if (!isPathCalibrated) {
                calibrateExecutionPath()
                isCalibrationFrame = true
            }

            // 2. Genuine On-Device YOLO Neural Inference
            val activeInterp = interpreter ?: interp
            outputBuffer.rewind()
            activeInterp.run(inputBuffer, outputBuffer)
            tInference = System.nanoTime()

            // 3. Post-processing: Decode [1, 84, 8400] output tensor
            outputFloatView.rewind()
            outputFloatView.get(flatOutput)
            tOutputCopy = System.nanoTime()

            val threshold = sensitivity.coerceIn(0.20f, 0.90f)
            val numClasses = min(NUM_CLASSES, labels.size)

            // Class argmax per anchor, walked class-major for sequential reads. Classes are
            // visited in ascending order and only a strictly greater score replaces the
            // incumbent, so ties resolve to the lowest class id exactly as before.
            java.util.Arrays.fill(bestScores, 0f)
            java.util.Arrays.fill(bestClasses, -1)
            for (c in 0 until numClasses) {
                val base = (4 + c) * NUM_ANCHORS
                for (a in 0 until NUM_ANCHORS) {
                    val score = flatOutput[base + a]
                    if (score > bestScores[a]) {
                        bestScores[a] = score
                        bestClasses[a] = c
                    }
                }
            }

            for (a in 0 until NUM_ANCHORS) {
                val maxClassScore = bestScores[a]
                val bestClassId = bestClasses[a]

                if (maxClassScore < threshold || bestClassId < 0) continue

                val rawLabel = if (bestClassId in labels.indices) labels[bestClassId] else continue
                val mapped = TfliteObjectDetector.mapCocoClass(rawLabel) ?: continue
                val (mappedType, semanticLabel) = mapped

                // YOLO11 coordinates are in letterboxed tensor pixel space: [cx, cy, w, h].
                // Undo the padding and scale to get coordinates in the upright frame.
                val box = FrameGeometry.toUprightNormalized(
                    centerX = flatOutput[0 * NUM_ANCHORS + a],
                    centerY = flatOutput[1 * NUM_ANCHORS + a],
                    width = flatOutput[2 * NUM_ANCHORS + a],
                    height = flatOutput[3 * NUM_ANCHORS + a],
                    letterbox = letterbox,
                    uprightWidth = uprightWidth,
                    uprightHeight = uprightHeight,
                ) ?: continue

                // Discard extreme noise boxes
                val boxWidth = box.width
                val boxHeight = box.height
                val area = boxWidth * boxHeight
                if (boxWidth < MIN_BOX_DIMENSION || boxHeight < MIN_BOX_DIMENSION) continue
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
        } catch (_: Exception) {
            // Perception resilience safeguard
        } finally {
            rawBitmap?.recycle()
        }
        tDecode = System.nanoTime()

        // 4. Class-Aware Non-Maximum Suppression (NMS)
        val nmsFiltered = TfliteObjectDetector.applyClassAwareNms(candidates, DEFAULT_NMS_IOU_THRESHOLD)

        // 5. Select top confident targets
        val topObjects = nmsFiltered
            .sortedByDescending { it.confidence }
            .take(MAX_REPORTED_OBJECTS)

        val tEnd = System.nanoTime()
        val duration = System.currentTimeMillis() - startTime

        return PerceptionFrameResult(
            detectedObjects = topObjects,
            visibilityScore = visibility,
            inferenceTimeMs = duration,
            frameWidth = uprightWidth,
            frameHeight = uprightHeight,
            timings = if (isCalibrationFrame) {
                null
            } else {
                PerceptionStageTimings(
                    visibilityUs = (tVisibility - t0) / 1000,
                    bitmapUs = (tBitmap - tVisibility) / 1000,
                    renderUs = (tRender - tBitmap) / 1000,
                    tensorFillUs = (tTensorFill - tRender) / 1000,
                    inferenceUs = (tInference - tTensorFill) / 1000,
                    outputCopyUs = (tOutputCopy - tInference) / 1000,
                    decodeUs = (tDecode - tOutputCopy) / 1000,
                    nmsUs = (tEnd - tDecode) / 1000,
                    totalUs = (tEnd - t0) / 1000,
                )
            },
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
        // Interpreter first: a delegate must not be released while still referenced.
        interpreter?.close()
        interpreter = null
        activeDelegate?.close()
        activeDelegate = null
        isPathCalibrated = false
        frameConverter?.recycle()
        frameConverter = null
        isLoaded = false
    }
}
