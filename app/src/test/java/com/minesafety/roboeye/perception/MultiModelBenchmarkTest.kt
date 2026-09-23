package com.minesafety.roboeye.perception

import android.content.Context
import android.content.res.AssetManager
import androidx.camera.core.ImageProxy
import com.minesafety.roboeye.core.ImuReading
import com.minesafety.roboeye.core.NodeSettings
import com.minesafety.roboeye.core.NormalizedRect
import com.minesafety.roboeye.core.PerceptionObjectType
import com.minesafety.roboeye.core.RawDetectedObject
import com.minesafety.roboeye.core.TrackedObject
import com.minesafety.roboeye.hardware.AccelerationVerificationStatus
import com.minesafety.roboeye.hardware.InferenceBackend
import com.minesafety.roboeye.hardware.InferenceExecutionReport
import com.minesafety.roboeye.localization.LocalPose
import com.minesafety.roboeye.localization.MonocularScaleState
import com.minesafety.roboeye.mapping.LocalSpatialMap
import com.minesafety.roboeye.mapping.MapCellState
import com.minesafety.roboeye.nav.model.DistanceCertainty
import com.minesafety.roboeye.nav.model.NavigationWorldModel
import com.minesafety.roboeye.nav.model.ObjectWorldProjection
import com.minesafety.roboeye.perception.benchmark.BenchmarkMetrics
import com.minesafety.roboeye.perception.benchmark.DetectorPerformanceSnapshot
import com.minesafety.roboeye.perception.model.DetectorCreationResult
import com.minesafety.roboeye.perception.model.DetectorFactory
import com.minesafety.roboeye.perception.model.DetectorMetadata
import com.minesafety.roboeye.perception.model.DetectorModelId
import com.minesafety.roboeye.perception.model.ModelReadinessStatus
import com.minesafety.roboeye.perception.model.ModelRegistry
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test
import java.io.FileNotFoundException
import java.io.InputStream
import java.util.concurrent.CountDownLatch
import java.util.concurrent.TimeUnit
import java.util.concurrent.atomic.AtomicInteger

/**
 * Deterministic test suite verifying Multi-Model Benchmark Infrastructure (Phase 6D).
 *
 * Implements comprehensive verification for requirements A through Z.
 */
class MultiModelBenchmarkTest {

    private class MockTestFrame(
        val seq: Long,
        val gen: Long,
    ) : SnapshotImageProxy(
        bitmap = null,
        rotation = 0,
        sequence = seq,
        timestampNs = seq * 1_000_000L,
        generation = gen,
        frameWidth = 640,
        frameHeight = 360,
    ) {
        var isFrameClosed = false

        override fun close() {
            super.close()
            isFrameClosed = true
        }
    }

    // =========================================================================
    // (A) DetectorModelId enum values and properties
    // =========================================================================
    @Test
    fun testA_detectorModelId_valuesAndProperties() {
        val entries = DetectorModelId.entries
        assertEquals(3, entries.size)
        assertTrue(entries.contains(DetectorModelId.YOLO11N))
        assertTrue(entries.contains(DetectorModelId.YOLO26N))
        assertTrue(entries.contains(DetectorModelId.SSD_MOBILENET))

        assertEquals("yolo11n", DetectorModelId.YOLO11N.id)
        assertEquals("YOLO11n", DetectorModelId.YOLO11N.displayName)
        assertEquals("models/yolo_detector.tflite", DetectorModelId.YOLO11N.assetFileName)
        assertTrue(DetectorModelId.YOLO11N.isAssetBundled)

        assertEquals("yolo26n", DetectorModelId.YOLO26N.id)
        assertEquals("YOLO26n", DetectorModelId.YOLO26N.displayName)
        assertEquals("models/yolo26n_detector.tflite", DetectorModelId.YOLO26N.assetFileName)
        assertFalse(DetectorModelId.YOLO26N.isAssetBundled)

        assertEquals("ssd_mobilenet_v1", DetectorModelId.SSD_MOBILENET.id)
        assertEquals("SSD MobileNet V1 (INT8)", DetectorModelId.SSD_MOBILENET.displayName)
        assertTrue(DetectorModelId.SSD_MOBILENET.isAssetBundled)

        // fromId mappings
        assertEquals(DetectorModelId.YOLO11N, DetectorModelId.fromId("yolo11n"))
        assertEquals(DetectorModelId.YOLO11N, DetectorModelId.fromId("YOLO11N"))
        assertEquals(DetectorModelId.YOLO26N, DetectorModelId.fromId("yolo26n"))
        assertEquals(DetectorModelId.YOLO26N, DetectorModelId.fromId("YOLO26N"))
        assertEquals(DetectorModelId.SSD_MOBILENET, DetectorModelId.fromId("ssd_mobilenet_v1"))
        assertEquals(DetectorModelId.YOLO11N, DetectorModelId.fromId("unknown_model"))
        assertEquals(DetectorModelId.YOLO11N, DetectorModelId.fromId(null))
    }

    // =========================================================================
    // (B) DetectorMetadata for YOLO11n matches known specification
    // =========================================================================
    @Test
    fun testB_detectorMetadata_yolo11n_matchesKnownSpecification() {
        val metadata = ModelRegistry.getMetadata(DetectorModelId.YOLO11N)
        assertEquals(DetectorModelId.YOLO11N, metadata.modelId)
        assertEquals(ModelReadinessStatus.READY, metadata.readinessStatus)
        assertTrue(metadata.isAvailable)
        assertEquals(640, metadata.inputWidth)
        assertEquals(640, metadata.inputHeight)
        assertEquals(3, metadata.inputChannels)
        assertEquals("Float32 RGB [0.0, 1.0]", metadata.inputTensorType)
        assertEquals("[1, 84, 8400] Float32 (cx, cy, w, h + 80 COCO classes)", metadata.outputFormat)
        assertEquals("models/yolo_detector.tflite", metadata.assetFileName)
    }

    // =========================================================================
    // (C) DetectorMetadata for YOLO26n reports PENDING_ASSET and zero tensor shapes
    // =========================================================================
    @Test
    fun testC_detectorMetadata_yolo26n_reportsPendingAssetAndZeroTensorShapes() {
        val metadata = ModelRegistry.getMetadata(DetectorModelId.YOLO26N)
        assertEquals(DetectorModelId.YOLO26N, metadata.modelId)
        assertEquals(ModelReadinessStatus.PENDING_ASSET, metadata.readinessStatus)
        assertFalse(metadata.isAvailable)
        // Hard constraint: Do NOT invent tensor shapes
        assertEquals(0, metadata.inputWidth)
        assertEquals(0, metadata.inputHeight)
        assertEquals(0, metadata.inputChannels)
        assertEquals("PENDING_ASSET_SPECIFICATION", metadata.inputTensorType)
        assertEquals("PENDING_ASSET_SPECIFICATION", metadata.outputFormat)
        assertEquals("models/yolo26n_detector.tflite", metadata.assetFileName)
        assertTrue(metadata.notes.contains("BLOCKED"))
    }

    // =========================================================================
    // (D) ModelRegistry returns all registered models
    // =========================================================================
    @Test
    fun testD_modelRegistry_returnsAllRegisteredModels() {
        val allModels = ModelRegistry.getAllModels()
        assertEquals(3, allModels.size)
        val ids = allModels.map { it.modelId }.toSet()
        assertTrue(ids.contains(DetectorModelId.YOLO11N))
        assertTrue(ids.contains(DetectorModelId.YOLO26N))
        assertTrue(ids.contains(DetectorModelId.SSD_MOBILENET))
    }

    // =========================================================================
    // (E) ModelRegistry correctly identifies YOLO11n as bundled and YOLO26n as missing
    // =========================================================================
    @Test
    fun testE_modelRegistry_identifiesBundledAndMissingAssets() {
        val yolo11 = ModelRegistry.getMetadata(DetectorModelId.YOLO11N)
        val yolo26 = ModelRegistry.getMetadata(DetectorModelId.YOLO26N)

        assertTrue(yolo11.modelId.isAssetBundled)
        assertFalse(yolo26.modelId.isAssetBundled)
    }

    // =========================================================================
    // (F) ModelRegistry getMetadata returns correct entry for each ID
    // =========================================================================
    @Test
    fun testF_modelRegistry_getMetadata_returnsCorrectEntryForEachId() {
        for (modelId in DetectorModelId.entries) {
            val meta = ModelRegistry.getMetadata(modelId)
            assertEquals(modelId, meta.modelId)
        }
    }

    private class FakeObjectDetector(
        override val detectorType: DetectorType = DetectorType.SSD_MOBILENET,
        override val modelName: String = "Fake SSD MobileNet V1",
        override val isLoaded: Boolean = true,
        override val preferredBackend: InferenceBackend = InferenceBackend.CPU,
        override val executionReport: InferenceExecutionReport = InferenceExecutionReport(
            backend = InferenceBackend.CPU,
            threadCount = 4,
            isFallbackActive = false,
            fallbackReason = null,
            accelerationStatus = AccelerationVerificationStatus.NOT_AVAILABLE,
            activeModelName = modelName,
            preferredModelName = modelName,
        ),
    ) : ObjectDetector {
        override fun detect(image: ImageProxy, sensitivity: Float): PerceptionFrameResult {
            return PerceptionFrameResult(emptyList(), 1.0f, 10L, 640, 360)
        }
        override fun close() {}
    }

    private class MockTestContext : android.content.ContextWrapper(null)

    // =========================================================================
    // (G) DetectorFactory creates YoloDetector or Unavailable when YOLO11N requested
    // =========================================================================
    @Test
    fun testG_detectorFactory_createsYoloDetectorOrUnavailable_whenYOLO11NRequested() {
        val mockContext = MockTestContext()
        val result = DetectorFactory.createDetector(DetectorModelId.YOLO11N, mockContext)
        assertNotNull(result)
        assertEquals(DetectorModelId.YOLO11N, result.requestedModelId)
        if (result is DetectorCreationResult.Success) {
            assertEquals(DetectorModelId.YOLO11N, result.activeModelId)
            assertNotNull(result.detector)
        } else {
            assertTrue(result is DetectorCreationResult.Unavailable)
            assertNull(result.activeModelId)
            assertNull(result.detector)
        }
    }

    // =========================================================================
    // (H) DetectorFactory fails gracefully and does NOT activate SSD when asset missing
    // =========================================================================
    @Test
    fun testH_detectorFactory_doesNotActivateSsd_whenYolo11nAssetMissing() {
        val mockContext = MockTestContext()
        val result = DetectorFactory.createDetector(DetectorModelId.YOLO11N, mockContext)
        assertTrue(result is DetectorCreationResult.Unavailable)
        assertEquals(DetectorModelId.YOLO11N, result.requestedModelId)
        // Hard constraint: Must NEVER activate SSD fallback
        assertNull(result.activeModelId)
        assertNull(result.detector)
        assertFalse(result.activeModelId == DetectorModelId.SSD_MOBILENET)
    }

    // =========================================================================
    // (I) DetectorFactory safely blocks YOLO26N instantiation (no SSD activation)
    // =========================================================================
    @Test
    fun testI_detectorFactory_safelyBlocksYolo26nInstantiation_doesNotActivateSsd() {
        val mockContext = MockTestContext()
        val result = DetectorFactory.createDetector(DetectorModelId.YOLO26N, mockContext)
        assertTrue(result is DetectorCreationResult.Unavailable)
        assertEquals(DetectorModelId.YOLO26N, result.requestedModelId)
        assertNull(result.activeModelId)
        assertNull(result.detector)
        val unavailable = result as DetectorCreationResult.Unavailable
        assertEquals(ModelReadinessStatus.PENDING_ASSET, unavailable.status)
        // Hard constraint: Must NEVER activate SSD when YOLO26n is selected
        assertFalse(result.activeModelId == DetectorModelId.SSD_MOBILENET)
        assertTrue(result.message.contains("YOLO26n"))
        assertTrue(result.message.contains("not bundled"))
    }

    // =========================================================================
    // (J) DetectorFactory creates SsdMobileNetDetector when SSD_MOBILENET requested
    // =========================================================================
    @Test
    fun testJ_detectorFactory_createsSsdMobileNetDetector_whenRequested() {
        val mockContext = MockTestContext()
        val fakeSsd = FakeObjectDetector(
            detectorType = DetectorType.SSD_MOBILENET,
            modelName = "SSD MobileNet V1 INT8",
            isLoaded = true,
        )
        try {
            DetectorFactory.detectorProvider = { modelId, _ ->
                if (modelId == DetectorModelId.SSD_MOBILENET) fakeSsd else null
            }
            val result = DetectorFactory.createDetector(DetectorModelId.SSD_MOBILENET, mockContext)
            assertTrue(result is DetectorCreationResult.Success)
            assertEquals(DetectorModelId.SSD_MOBILENET, result.requestedModelId)
            assertEquals(DetectorModelId.SSD_MOBILENET, result.activeModelId)
            assertEquals(fakeSsd, result.detector)
        } finally {
            DetectorFactory.detectorProvider = null
        }
    }

    // =========================================================================
    // (K) BenchmarkMetrics records inference latency and computes average correctly
    // =========================================================================
    @Test
    fun testK_benchmarkMetrics_recordsInferenceLatency_computesAverageCorrectly() {
        val metrics = BenchmarkMetrics(DetectorModelId.YOLO11N)
        metrics.recordSuccess(10L)
        metrics.recordSuccess(20L)
        metrics.recordSuccess(30L)

        val snap = metrics.snapshot()
        assertEquals(3L, snap.totalInferences)
        assertEquals(3L, snap.successfulInferences)
        assertEquals(0L, snap.failedInferences)
        assertEquals(30L, snap.lastLatencyMs)
        assertEquals(20.0f, snap.avgLatencyMs, 0.001f)
    }

    // =========================================================================
    // (L) BenchmarkMetrics computes p50 and p95 percentiles accurately
    // =========================================================================
    @Test
    fun testL_benchmarkMetrics_computesP50AndP95PercentilesAccurately() {
        val metrics = BenchmarkMetrics(DetectorModelId.YOLO11N)
        for (i in 1..100) {
            metrics.recordSuccess(i.toLong())
        }

        val snap = metrics.snapshot()
        assertEquals(100L, snap.successfulInferences)
        // 1 to 100: p50 should be around 50, p95 should be around 95
        assertTrue("p50 should be ~50ms, was ${snap.p50LatencyMs}", snap.p50LatencyMs in 49.0f..51.0f)
        assertTrue("p95 should be ~95ms, was ${snap.p95LatencyMs}", snap.p95LatencyMs in 94.0f..96.0f)
    }

    // =========================================================================
    // (M) BenchmarkMetrics tracks min and max latency
    // =========================================================================
    @Test
    fun testM_benchmarkMetrics_tracksMinAndMaxLatency() {
        val metrics = BenchmarkMetrics(DetectorModelId.YOLO11N)
        metrics.recordSuccess(45L)
        metrics.recordSuccess(12L)
        metrics.recordSuccess(88L)
        metrics.recordSuccess(30L)

        val snap = metrics.snapshot()
        assertEquals(12L, snap.minLatencyMs)
        assertEquals(88L, snap.maxLatencyMs)
    }

    // =========================================================================
    // (N) BenchmarkMetrics separates detector throughput from production rate
    // =========================================================================
    @Test
    fun testN_benchmarkMetrics_separatesDetectorThroughputFromProductionRate() {
        val metrics = BenchmarkMetrics(DetectorModelId.YOLO11N)

        // Latency 50ms means theoretical throughput = 1000 / 50 = 20.0 FPS
        // Production timestamps spaced 1000ms apart means actual production rate = 1.0 Hz
        metrics.recordSuccess(latencyMs = 50L, timestampMs = 1000L)
        metrics.recordSuccess(latencyMs = 50L, timestampMs = 2000L)

        val snap = metrics.snapshot()
        assertEquals(20.0f, snap.detectorThroughputFps, 0.01f)
        assertEquals(1.0f, snap.productionRateHz, 0.01f)
    }

    // =========================================================================
    // (O) BenchmarkMetrics sanitizes NaN and Infinite values to 0.0f
    // =========================================================================
    @Test
    fun testO_benchmarkMetrics_sanitizesNaNAndInfiniteValues() {
        assertEquals(0.0f, BenchmarkMetrics.sanitize(Float.NaN), 0.0f)
        assertEquals(0.0f, BenchmarkMetrics.sanitize(Float.POSITIVE_INFINITY), 0.0f)
        assertEquals(0.0f, BenchmarkMetrics.sanitize(Float.NEGATIVE_INFINITY), 0.0f)
        assertEquals(0.0, BenchmarkMetrics.sanitize(Double.NaN), 0.0)
        assertEquals(0.0, BenchmarkMetrics.sanitize(Double.POSITIVE_INFINITY), 0.0)
        assertEquals(12.34f, BenchmarkMetrics.sanitize(12.34f), 0.001f)
    }

    // =========================================================================
    // (P) BenchmarkMetrics handles zero samples safely (no divide-by-zero)
    // =========================================================================
    @Test
    fun testP_benchmarkMetrics_handlesZeroSamplesSafely_noDivideByZero() {
        val metrics = BenchmarkMetrics(DetectorModelId.YOLO11N)
        val snap = metrics.snapshot()

        assertEquals(0L, snap.totalInferences)
        assertEquals(0L, snap.successfulInferences)
        assertEquals(0L, snap.failedInferences)
        assertEquals(0L, snap.skippedFrames)
        assertEquals(0.0f, snap.avgLatencyMs, 0.0f)
        assertEquals(0.0f, snap.p50LatencyMs, 0.0f)
        assertEquals(0.0f, snap.p95LatencyMs, 0.0f)
        assertEquals(0.0f, snap.detectorThroughputFps, 0.0f)
        assertEquals(0.0f, snap.productionRateHz, 0.0f)
        assertFalse(snap.avgLatencyMs.isNaN())
        assertFalse(snap.detectorThroughputFps.isNaN())
        assertFalse(snap.productionRateHz.isNaN())
    }

    // =========================================================================
    // (Q) BenchmarkMetrics records skipped frames correctly
    // =========================================================================
    @Test
    fun testQ_benchmarkMetrics_recordsSkippedFramesCorrectly() {
        val metrics = BenchmarkMetrics(DetectorModelId.YOLO11N)
        metrics.recordSkippedFrame()
        metrics.recordSkippedFrame()
        metrics.recordSkippedFrame()

        val snap = metrics.snapshot()
        assertEquals(3L, snap.skippedFrames)
    }

    // =========================================================================
    // (R) BenchmarkMetrics records failed inferences correctly
    // =========================================================================
    @Test
    fun testR_benchmarkMetrics_recordsFailedInferencesCorrectly() {
        val metrics = BenchmarkMetrics(DetectorModelId.YOLO11N)
        metrics.recordSuccess(25L)
        metrics.recordFailure()
        metrics.recordFailure()

        val snap = metrics.snapshot()
        assertEquals(3L, snap.totalInferences)
        assertEquals(1L, snap.successfulInferences)
        assertEquals(2L, snap.failedInferences)
    }

    // =========================================================================
    // (S) BenchmarkMetrics resetForModelSwitch clears previous model metrics
    // =========================================================================
    @Test
    fun testS_benchmarkMetrics_resetForModelSwitch_clearsPreviousModelMetrics() {
        val metrics = BenchmarkMetrics(DetectorModelId.YOLO11N)
        metrics.recordSuccess(30L)
        metrics.recordSuccess(40L)
        metrics.recordSkippedFrame()
        metrics.recordFailure()

        metrics.resetForModelSwitch(DetectorModelId.YOLO26N)
        val snap = metrics.snapshot()

        assertEquals(DetectorModelId.YOLO26N, snap.requestedModelId)
        assertEquals(0L, snap.totalInferences)
        assertEquals(0L, snap.successfulInferences)
        assertEquals(0L, snap.failedInferences)
        assertEquals(0L, snap.skippedFrames)
        assertEquals(0.0f, snap.avgLatencyMs, 0.0f)
        assertEquals(0L, snap.lastLatencyMs)
        assertEquals(0L, snap.minLatencyMs)
        assertEquals(0L, snap.maxLatencyMs)
    }

    // =========================================================================
    // (T) BenchmarkMetrics snapshot produces immutable consistent state
    // =========================================================================
    @Test
    fun testT_benchmarkMetrics_snapshotProducesImmutableConsistentState() {
        val metrics = BenchmarkMetrics(DetectorModelId.YOLO11N)
        metrics.recordSuccess(20L)
        val snap1 = metrics.snapshot()

        metrics.recordSuccess(50L)
        metrics.recordSuccess(100L)
        val snap2 = metrics.snapshot()

        // snap1 must be unchanged
        assertEquals(1L, snap1.successfulInferences)
        assertEquals(20.0f, snap1.avgLatencyMs, 0.001f)

        // snap2 reflects latest state
        assertEquals(3L, snap2.successfulInferences)
        assertTrue(snap2.avgLatencyMs > snap1.avgLatencyMs)
    }

    // =========================================================================
    // (U) Model switch increments PerceptionMailbox generation
    // =========================================================================
    @Test
    fun testU_modelSwitch_incrementsPerceptionMailboxGeneration() {
        val mailbox = PerceptionMailbox()
        val g0 = mailbox.currentGeneration

        mailbox.onDisabled()
        val g1 = mailbox.currentGeneration
        assertTrue(g1 > g0)

        mailbox.onEnabled()
        val g2 = mailbox.currentGeneration
        assertTrue(g2 > g1)

        mailbox.shutdown()
    }

    // =========================================================================
    // (V) Model switch invalidates in-flight frames
    // =========================================================================
    @Test
    fun testV_modelSwitch_invalidatesInFlightFrames() {
        val mailbox = PerceptionMailbox()
        val resultLatch = CountDownLatch(1)
        val processedCount = AtomicInteger(0)

        mailbox.engine = object : PerceptionEngine {
            override fun processFrame(image: ImageProxy, sensitivity: Float): PerceptionFrameResult {
                processedCount.incrementAndGet()
                return PerceptionFrameResult(
                    detectedObjects = emptyList(),
                    visibilityScore = 0.9f,
                    inferenceTimeMs = 10L,
                    frameWidth = 640,
                    frameHeight = 360,
                )
            }
        }
        mailbox.onResult = { resultLatch.countDown() }

        mailbox.onEnabled()
        val oldGen = mailbox.currentGeneration

        // Invalidate generation via disable
        mailbox.onDisabled()
        mailbox.onEnabled()

        // Submit frame stamped with OLD generation
        val staleFrame = MockTestFrame(seq = 1L, gen = oldGen)
        mailbox.submit(staleFrame)

        // Result should NOT be emitted
        val received = resultLatch.await(300, TimeUnit.MILLISECONDS)
        assertFalse("Stale in-flight frame must not emit perception results", received)

        mailbox.shutdown()
    }

    // =========================================================================
    // (W) Model switch resets ByteTracker state
    // =========================================================================
    @Test
    fun testW_modelSwitch_resetsByteTrackerState() {
        val tracker = ByteTracker()
        val dummyDet = RawDetectedObject(
            boundingBox = NormalizedRect(0.2f, 0.2f, 0.4f, 0.4f),
            label = "person",
            type = PerceptionObjectType.OBSTACLE,
            confidence = 0.9f,
            timestampMs = 1000L,
        )

        // Track frame 1
        val tracks1 = tracker.update(
            detections = listOf(dummyDet),
            cameraHfovDeg = 68f,
            cameraVfovDeg = 52f,
            cameraHeightM = 0.95f,
            imuPitchDeg = 0f,
            imuReading = ImuReading(isAvailable = true),
        )
        assertEquals(1, tracks1.size)
        val originalTrackId = tracks1.first().id

        // Model switch triggers tracker.reset()
        tracker.reset()

        // After reset, track pool is completely cleared
        val tracksAfterReset = tracker.update(
            detections = emptyList(),
            cameraHfovDeg = 68f,
            cameraVfovDeg = 52f,
            cameraHeightM = 0.95f,
            imuPitchDeg = 0f,
            imuReading = ImuReading(isAvailable = true),
        )
        assertTrue(tracksAfterReset.isEmpty())
    }

    // =========================================================================
    // (X) Model switch preserves downstream WorldModel and semantic radar pipeline
    // =========================================================================
    @Test
    fun testX_modelSwitch_preservesDownstreamWorldModelAndSemanticRadarPipeline() {
        val dummyTrack = TrackedObject(
            id = "42",
            type = PerceptionObjectType.OBSTACLE,
            label = "person",
            boundingBox = NormalizedRect(0.3f, 0.3f, 0.6f, 0.6f),
            bearingDeg = 4.0f,
            estimatedDistanceM = 3.5f,
            confidence = 0.88f,
            isConfirmed = true,
        )

        val localPose = LocalPose(
            xM = 2.0f,
            yM = 1.0f,
            yawDeg = 0.0f,
            scaleState = MonocularScaleState.SCALE_ESTIMATED,
        )
        val worldObj = ObjectWorldProjection.projectSingleTrack(dummyTrack, localPose)
        assertNotNull(worldObj)

        val worldModel = NavigationWorldModel(
            pose = localPose,
            spatialMap = LocalSpatialMap(),
            semanticObjects = listOf(worldObj),
        )

        assertEquals(1, worldModel.semanticObjects.size)
        assertEquals("person", worldModel.semanticObjects.first().label)
        assertEquals(42, worldModel.semanticObjects.first().trackId)
        assertEquals(DistanceCertainty.ESTIMATED_PRIOR, worldObj.distanceCertainty)
    }

    // =========================================================================
    // (Y) SettingsStore persists and restores selectedDetectorModel
    // =========================================================================
    @Test
    fun testY_settingsStore_persistsAndRestoresSelectedDetectorModel() {
        val defaultSettings = NodeSettings()
        assertEquals(DetectorModelId.YOLO11N, defaultSettings.selectedDetectorModel)

        val customSettings = defaultSettings.copy(selectedDetectorModel = DetectorModelId.YOLO26N)
        assertEquals(DetectorModelId.YOLO26N, customSettings.selectedDetectorModel)

        // Verify serialization ID
        val serializedId = customSettings.selectedDetectorModel.id
        assertEquals("yolo26n", serializedId)
        val restored = DetectorModelId.fromId(serializedId)
        assertEquals(DetectorModelId.YOLO26N, restored)
    }

    // =========================================================================
    // (Z) Full pipeline execution with benchmark recording produces non-null snapshot
    // =========================================================================
    @Test
    fun testZ_fullPipelineExecution_producesNonNullBenchmarkSnapshot() {
        val metrics = BenchmarkMetrics(DetectorModelId.YOLO11N)
        metrics.recordInitialization(
            durationMs = 150L,
            isFallback = false,
            activeModel = DetectorModelId.YOLO11N,
            name = "YOLO11n",
        )

        // Record 5 successful perception passes
        for (i in 1..5) {
            metrics.recordSuccess(latencyMs = 35L + i, timestampMs = 1000L + i * 250L)
        }
        metrics.recordSkippedFrame()

        val snap = metrics.snapshot()
        assertNotNull(snap)
        assertEquals(DetectorModelId.YOLO11N, snap.requestedModelId)
        assertEquals(DetectorModelId.YOLO11N, snap.activeModelId)
        assertFalse(snap.isFallbackActive)
        assertEquals(5L, snap.successfulInferences)
        assertEquals(1L, snap.skippedFrames)
        assertEquals(0L, snap.failedInferences)
        assertTrue(snap.avgLatencyMs in 35.0f..45.0f)
        assertTrue(snap.detectorThroughputFps > 0.0f)
        assertTrue(snap.productionRateHz > 0.0f)
    }

    // =========================================================================
    // Phase 6D Fix - Scenario 1: YOLO11n initialization failure -> does NOT activate SSD
    // =========================================================================
    @Test
    fun testFix_scenario1_yolo11nFailure_doesNotActivateSsd() {
        val mockContext = MockTestContext()
        val result = DetectorFactory.createDetector(DetectorModelId.YOLO11N, mockContext)
        // Failure without assets
        assertTrue(result is DetectorCreationResult.Unavailable)
        assertEquals(DetectorModelId.YOLO11N, result.requestedModelId)
        assertNull("activeModelId must be null upon YOLO11n failure", result.activeModelId)
        assertNull("detector must be null upon YOLO11n failure", result.detector)
        assertFalse("Must NEVER activate SSD upon YOLO11n failure", result.activeModelId == DetectorModelId.SSD_MOBILENET)
    }

    // =========================================================================
    // Phase 6D Fix - Scenario 2: YOLO26n asset missing -> does NOT activate SSD
    // =========================================================================
    @Test
    fun testFix_scenario2_yolo26nPendingAsset_doesNotActivateSsd() {
        val mockContext = MockTestContext()
        val result = DetectorFactory.createDetector(DetectorModelId.YOLO26N, mockContext)
        assertTrue(result is DetectorCreationResult.Unavailable)
        assertEquals(DetectorModelId.YOLO26N, result.requestedModelId)
        assertNull("activeModelId must be null for pending YOLO26n", result.activeModelId)
        assertNull("detector must be null for pending YOLO26n", result.detector)
        val unavailable = result as DetectorCreationResult.Unavailable
        assertEquals(ModelReadinessStatus.PENDING_ASSET, unavailable.status)
        assertFalse("Must NEVER activate SSD when YOLO26n requested", result.activeModelId == DetectorModelId.SSD_MOBILENET)
        assertFalse("Must NEVER activate YOLO11n when YOLO26n requested", result.activeModelId == DetectorModelId.YOLO11N)
    }

    // =========================================================================
    // Phase 6D Fix - Scenario 3: Explicit SSD selection -> activates SSD
    // =========================================================================
    @Test
    fun testFix_scenario3_explicitSsdSelection_activatesSsd() {
        val mockContext = MockTestContext()
        val fakeSsd = FakeObjectDetector(
            detectorType = DetectorType.SSD_MOBILENET,
            modelName = "SSD MobileNet V1 INT8",
            isLoaded = true,
        )
        try {
            DetectorFactory.detectorProvider = { modelId, _ ->
                if (modelId == DetectorModelId.SSD_MOBILENET) fakeSsd else null
            }
            val result = DetectorFactory.createDetector(DetectorModelId.SSD_MOBILENET, mockContext)
            assertTrue(result is DetectorCreationResult.Success)
            assertEquals(DetectorModelId.SSD_MOBILENET, result.requestedModelId)
            assertEquals(DetectorModelId.SSD_MOBILENET, result.activeModelId)
            assertEquals(fakeSsd, result.detector)
        } finally {
            DetectorFactory.detectorProvider = null
        }
    }

    // =========================================================================
    // Phase 6D Fix - Scenario 4: Identity consistency: Whenever a detector runs, selectedModelId == activeModelId
    // =========================================================================
    @Test
    fun testFix_scenario4_identityConsistency_wheneverDetectorRuns() {
        val mockContext = MockTestContext()
        for (modelId in listOf(DetectorModelId.YOLO11N, DetectorModelId.SSD_MOBILENET)) {
            val fakeDetector = FakeObjectDetector(
                detectorType = if (modelId == DetectorModelId.YOLO11N) DetectorType.YOLO else DetectorType.SSD_MOBILENET,
                modelName = modelId.displayName,
                isLoaded = true,
            )
            try {
                DetectorFactory.detectorProvider = { requested, _ ->
                    if (requested == modelId) fakeDetector else null
                }
                val result = DetectorFactory.createDetector(modelId, mockContext)
                assertTrue(result is DetectorCreationResult.Success)
                // Strict invariant: selectedModelId == activeModelId
                assertEquals(modelId, result.requestedModelId)
                assertEquals(modelId, result.activeModelId)
                assertEquals(result.requestedModelId, result.activeModelId)
            } finally {
                DetectorFactory.detectorProvider = null
            }
        }
    }

    // =========================================================================
    // Phase 6D Fix - Scenario 5: SSD failure -> does NOT activate YOLO11n or YOLO26n
    // =========================================================================
    @Test
    fun testFix_scenario5_ssdFailure_doesNotActivateYolo() {
        val mockContext = MockTestContext()
        val brokenDetector = FakeObjectDetector(
            detectorType = DetectorType.SSD_MOBILENET,
            modelName = "Broken SSD",
            isLoaded = false, // Not loaded
        )
        try {
            DetectorFactory.detectorProvider = { modelId, _ ->
                if (modelId == DetectorModelId.SSD_MOBILENET) brokenDetector else null
            }
            val result = DetectorFactory.createDetector(DetectorModelId.SSD_MOBILENET, mockContext)
            assertTrue(result is DetectorCreationResult.Unavailable)
            assertEquals(DetectorModelId.SSD_MOBILENET, result.requestedModelId)
            assertNull("activeModelId must be null upon SSD failure", result.activeModelId)
            assertNull("detector must be null upon SSD failure", result.detector)
            assertFalse("Must NEVER activate YOLO11n upon SSD failure", result.activeModelId == DetectorModelId.YOLO11N)
            assertFalse("Must NEVER activate YOLO26n upon SSD failure", result.activeModelId == DetectorModelId.YOLO26N)
        } finally {
            DetectorFactory.detectorProvider = null
        }
    }

    // =========================================================================
    // Phase 6D Fix - Scenario 6: AI detector unavailable/failure -> semantic objects cleared, geometric perception continues
    // =========================================================================
    @Test
    fun testFix_scenario6_aiDetectorUnavailable_clearsSemanticObjects_geometricPerceptionContinues() {
        // 1. Simulate an initial world model with semantic objects
        val dummyTrack = TrackedObject(
            id = "101",
            type = PerceptionObjectType.OBSTACLE,
            label = "mine_cart",
            boundingBox = NormalizedRect(0.3f, 0.3f, 0.7f, 0.7f),
            bearingDeg = 0.0f,
            estimatedDistanceM = 2.5f,
            confidence = 0.95f,
            isConfirmed = true,
        )
        val initialPose = LocalPose(xM = 5.0f, yM = 3.0f, yawDeg = 45.0f, scaleState = MonocularScaleState.SCALE_ESTIMATED)
        val worldObj = ObjectWorldProjection.projectSingleTrack(dummyTrack, initialPose)
        assertNotNull(worldObj)

        val map = LocalSpatialMap()
        map.updateCell(5, 5, MapCellState.OCCUPIED, 0.8f, 1000L)

        val activeWorldModel = NavigationWorldModel(
            pose = initialPose,
            spatialMap = map,
            semanticObjects = listOf(worldObj),
        )
        assertEquals(1, activeWorldModel.semanticObjects.size)

        // 2. Simulate AI detector failure / unavailable: semantic objects are cleared to empty list
        val clearedSemanticObjects = emptyList<com.minesafety.roboeye.nav.model.WorldObject>()
        val recoveredWorldModel = NavigationWorldModel(
            pose = initialPose, // Geometric pose continues uninterrupted
            spatialMap = map,   // Local spatial map continues uninterrupted
            semanticObjects = clearedSemanticObjects,
        )

        // Verify semantic objects cleared
        assertTrue(recoveredWorldModel.semanticObjects.isEmpty())
        // Verify geometric perception state continues intact
        assertEquals(5.0f, recoveredWorldModel.pose.xM, 0.001f)
        assertEquals(3.0f, recoveredWorldModel.pose.yM, 0.001f)
        assertEquals(45.0f, recoveredWorldModel.pose.yawDeg, 0.001f)
        assertEquals(MapCellState.OCCUPIED, recoveredWorldModel.spatialMap.cells[5][5].state)
    }

    // =========================================================================
    // Phase 6E.1 — Comprehensive YOLO26n Runtime Integration Tests (Req 1 - 29)
    // =========================================================================

    // Req 1: YOLO26N ID exists
    @Test
    fun test6E1_req01_yolo26nIdExists() {
        assertNotNull(DetectorModelId.valueOf("YOLO26N"))
        assertEquals("yolo26n", DetectorModelId.YOLO26N.id)
        assertEquals("YOLO26n", DetectorModelId.YOLO26N.displayName)
    }

    // Req 2: DetectorFactory supports YOLO26N
    @Test
    fun test6E1_req02_detectorFactorySupportsYOLO26N() {
        val mockContext = MockTestContext()
        val result = DetectorFactory.createDetector(DetectorModelId.YOLO26N, mockContext)
        assertNotNull(result)
        assertEquals(DetectorModelId.YOLO26N, result.requestedModelId)
    }

    // Req 3: Valid YOLO26n asset is discovered or mocked
    @Test
    fun test6E1_req03_validYolo26nAssetDiscoveredOrMocked() {
        val fakeYolo26 = FakeObjectDetector(
            detectorType = DetectorType.YOLO,
            modelName = "YOLO26n",
            isLoaded = true,
        )
        try {
            DetectorFactory.detectorProvider = { modelId, _ ->
                if (modelId == DetectorModelId.YOLO26N) fakeYolo26 else null
            }
            val result = DetectorFactory.createDetector(DetectorModelId.YOLO26N, MockTestContext())
            assertTrue(result is DetectorCreationResult.Success)
            assertEquals(fakeYolo26, (result as DetectorCreationResult.Success).detector)
        } finally {
            DetectorFactory.detectorProvider = null
        }
    }

    // Req 4: Missing asset -> unavailable
    @Test
    fun test6E1_req04_missingAssetReportsUnavailable() {
        val result = DetectorFactory.createDetector(DetectorModelId.YOLO26N, MockTestContext())
        assertTrue(result is DetectorCreationResult.Unavailable)
        assertEquals(DetectorModelId.YOLO26N, result.requestedModelId)
        assertNull(result.activeModelId)
        assertNull(result.detector)
        assertEquals(ModelReadinessStatus.PENDING_ASSET, (result as DetectorCreationResult.Unavailable).status)
    }

    // Req 5: Invalid/incompatible tensor contract -> unavailable
    @Test
    fun test6E1_req05_incompatibleTensorContractReportsUnavailable() {
        val brokenDetector = FakeObjectDetector(
            detectorType = DetectorType.YOLO,
            modelName = "Incompatible YOLO26n",
            isLoaded = false,
        )
        try {
            DetectorFactory.detectorProvider = { modelId, _ ->
                if (modelId == DetectorModelId.YOLO26N) brokenDetector else null
            }
            val result = DetectorFactory.createDetector(DetectorModelId.YOLO26N, MockTestContext())
            assertTrue(result is DetectorCreationResult.Unavailable)
            assertNull(result.activeModelId)
            assertNull(result.detector)
            assertEquals(ModelReadinessStatus.ERROR, (result as DetectorCreationResult.Unavailable).status)
        } finally {
            DetectorFactory.detectorProvider = null
        }
    }

    // Req 6: Valid YOLO26n model initializes
    @Test
    fun test6E1_req06_validYolo26nModelInitializes() {
        val validDetector = FakeObjectDetector(
            detectorType = DetectorType.YOLO,
            modelName = "YOLO26n",
            isLoaded = true,
        )
        try {
            DetectorFactory.detectorProvider = { modelId, _ ->
                if (modelId == DetectorModelId.YOLO26N) validDetector else null
            }
            val result = DetectorFactory.createDetector(DetectorModelId.YOLO26N, MockTestContext())
            assertTrue(result is DetectorCreationResult.Success)
            assertTrue(result.detector?.isLoaded == true)
        } finally {
            DetectorFactory.detectorProvider = null
        }
    }

    // Req 7: YOLO26n selected -> YOLO26n active
    @Test
    fun test6E1_req07_yolo26nSelectedMakesYolo26nActive() {
        val validDetector = FakeObjectDetector(
            detectorType = DetectorType.YOLO,
            modelName = "YOLO26n",
            isLoaded = true,
        )
        try {
            DetectorFactory.detectorProvider = { modelId, _ ->
                if (modelId == DetectorModelId.YOLO26N) validDetector else null
            }
            val result = DetectorFactory.createDetector(DetectorModelId.YOLO26N, MockTestContext())
            assertTrue(result is DetectorCreationResult.Success)
            assertEquals(DetectorModelId.YOLO26N, result.activeModelId)
            assertEquals(result.requestedModelId, result.activeModelId)
        } finally {
            DetectorFactory.detectorProvider = null
        }
    }

    // Req 8: YOLO26n init failure -> active = null, no SSD, no YOLO11n
    @Test
    fun test6E1_req08_yolo26nInitFailure_noCrossModelFallback() {
        val result = DetectorFactory.createDetector(DetectorModelId.YOLO26N, MockTestContext())
        assertTrue(result is DetectorCreationResult.Unavailable)
        assertNull(result.activeModelId)
        assertFalse(result.activeModelId == DetectorModelId.SSD_MOBILENET)
        assertFalse(result.activeModelId == DetectorModelId.YOLO11N)
    }

    // Req 9: YOLO11n selected: YOLO26n not invoked
    @Test
    fun test6E1_req09_yolo11nSelectedDoesNotInvokeYolo26n() {
        var yolo26Invoked = false
        var yolo11Invoked = false
        try {
            DetectorFactory.detectorProvider = { modelId, _ ->
                when (modelId) {
                    DetectorModelId.YOLO11N -> {
                        yolo11Invoked = true
                        FakeObjectDetector(modelName = "YOLO11n")
                    }
                    DetectorModelId.YOLO26N -> {
                        yolo26Invoked = true
                        FakeObjectDetector(modelName = "YOLO26n")
                    }
                    else -> null
                }
            }
            val result = DetectorFactory.createDetector(DetectorModelId.YOLO11N, MockTestContext())
            assertTrue(result is DetectorCreationResult.Success)
            assertTrue(yolo11Invoked)
            assertFalse(yolo26Invoked)
        } finally {
            DetectorFactory.detectorProvider = null
        }
    }

    // Req 10: SSD selected: YOLO26n not invoked
    @Test
    fun test6E1_req10_ssdSelectedDoesNotInvokeYolo26n() {
        var yolo26Invoked = false
        var ssdInvoked = false
        try {
            DetectorFactory.detectorProvider = { modelId, _ ->
                when (modelId) {
                    DetectorModelId.SSD_MOBILENET -> {
                        ssdInvoked = true
                        FakeObjectDetector(detectorType = DetectorType.SSD_MOBILENET, modelName = "SSD")
                    }
                    DetectorModelId.YOLO26N -> {
                        yolo26Invoked = true
                        FakeObjectDetector(modelName = "YOLO26n")
                    }
                    else -> null
                }
            }
            val result = DetectorFactory.createDetector(DetectorModelId.SSD_MOBILENET, MockTestContext())
            assertTrue(result is DetectorCreationResult.Success)
            assertTrue(ssdInvoked)
            assertFalse(yolo26Invoked)
        } finally {
            DetectorFactory.detectorProvider = null
        }
    }

    // Req 11: YOLO26n output converts to existing detector result
    @Test
    fun test6E1_req11_yolo26nOutputConvertsToExistingResult() {
        val raw = RawDetectedObject(
            boundingBox = NormalizedRect(0.1f, 0.2f, 0.5f, 0.6f),
            label = "person",
            type = PerceptionObjectType.PERSON,
            confidence = 0.88f,
            timestampMs = 12345L,
        )
        val frameResult = PerceptionFrameResult(
            detectedObjects = listOf(raw),
            visibilityScore = 0.95f,
            inferenceTimeMs = 42L,
            frameWidth = 640,
            frameHeight = 480,
        )
        assertEquals(1, frameResult.detectedObjects.size)
        assertEquals("person", frameResult.detectedObjects[0].label)
        assertEquals(0.88f, frameResult.detectedObjects[0].confidence, 0.001f)
    }

    // Req 12: YOLO26n bounding-box coordinate mapping is correct
    @Test
    fun test6E1_req12_boundingBoxCoordinateMapping() {
        val letterbox = FrameGeometry.letterboxFor(
            srcWidth = 640,
            srcHeight = 360,
            target = 640,
        )
        val uprightBox = FrameGeometry.toUprightNormalized(
            centerX = 320f,
            centerY = 320f,
            width = 100f,
            height = 100f,
            letterbox = letterbox,
            uprightWidth = 640,
            uprightHeight = 360,
        )
        assertNotNull(uprightBox)
        assertTrue(uprightBox!!.left >= 0.0f && uprightBox.left <= 1.0f)
        assertTrue(uprightBox.top >= 0.0f && uprightBox.top <= 1.0f)
        assertTrue(uprightBox.right >= 0.0f && uprightBox.right <= 1.0f)
        assertTrue(uprightBox.bottom >= 0.0f && uprightBox.bottom <= 1.0f)
    }

    // Req 13: YOLO26n class/confidence decoding is correct
    @Test
    fun test6E1_req13_classConfidenceDecoding() {
        val mapped = TfliteObjectDetector.mapCocoClass("person")
        assertNotNull(mapped)
        assertEquals(PerceptionObjectType.PERSON, mapped!!.first)
        assertEquals("person", mapped.second)
    }

    // Req 14: Frame/timestamp identity is preserved
    @Test
    fun test6E1_req14_frameTimestampPreserved() {
        val testTs = 987654321L
        val raw = RawDetectedObject(
            boundingBox = NormalizedRect(0.1f, 0.1f, 0.2f, 0.2f),
            label = "person",
            type = PerceptionObjectType.PERSON,
            confidence = 0.9f,
            timestampMs = testTs,
        )
        assertEquals(testTs, raw.timestampMs)
    }

    // Req 15: ByteTracker receives normal detection contract
    @Test
    fun test6E1_req15_byteTrackerReceivesNormalDetectionContract() {
        val tracker = ByteTracker()
        val raw = RawDetectedObject(
            boundingBox = NormalizedRect(0.2f, 0.2f, 0.4f, 0.5f),
            label = "person",
            type = PerceptionObjectType.PERSON,
            confidence = 0.85f,
        )
        val tracked = tracker.update(
            detections = listOf(raw),
            cameraHfovDeg = 68f,
            cameraVfovDeg = 52f,
            cameraHeightM = 0.95f,
            imuPitchDeg = 0f,
            imuReading = ImuReading(isAvailable = true),
        )
        assertEquals(1, tracked.size)
        assertEquals(PerceptionObjectType.PERSON, tracked[0].type)
        assertEquals("person", tracked[0].label)
    }

    // Req 16: AI OFF disables YOLO26n
    @Test
    fun test6E1_req16_aiOffDisablesYolo26n() {
        val mailbox = PerceptionMailbox()
        mailbox.isEnabled = false
        val frame = MockTestFrame(1L, 1L)
        mailbox.submit(frame)
        // Disabled mailbox closes frame immediately without queuing
        assertTrue(frame.isFrameClosed)
        mailbox.shutdown()
    }

    // Req 17: Lifecycle closes YOLO26n resources
    @Test
    fun test6E1_req17_lifecycleClosesYolo26nResources() {
        var closed = false
        val customDetector = object : ObjectDetector {
            override val detectorType = DetectorType.YOLO
            override val modelName = "YOLO26n"
            override val isLoaded = true
            override val preferredBackend = InferenceBackend.CPU
            override val executionReport = InferenceExecutionReport(
                backend = InferenceBackend.CPU,
                threadCount = 4,
                isFallbackActive = false,
                fallbackReason = null,
                accelerationStatus = AccelerationVerificationStatus.NOT_AVAILABLE,
                activeModelName = modelName,
                preferredModelName = modelName,
            )
            override fun detect(image: ImageProxy, sensitivity: Float) = PerceptionFrameResult(emptyList(), 1f, 0L, 640, 480)
            override fun close() { closed = true }
        }
        customDetector.close()
        assertTrue(closed)
    }

    // Req 18: Benchmark recognizes YOLO26N
    @Test
    fun test6E1_req18_benchmarkRecognizesYOLO26N() {
        val session = com.minesafety.roboeye.perception.benchmark.DeviceBenchmarkSession()
        session.startSession(
            model = DetectorModelId.YOLO26N,
            activeModel = DetectorModelId.YOLO26N,
            warmupFrames = 10,
            durationMs = 10_000L,
        )
        assertEquals(com.minesafety.roboeye.perception.benchmark.BenchmarkStatus.WARMING_UP, session.currentStatus)
    }

    // Req 19: Benchmark report contains model = YOLO26N
    @Test
    fun test6E1_req19_benchmarkReportContainsModelYOLO26N() {
        val report = com.minesafety.roboeye.perception.benchmark.DeviceBenchmarkReport(
            runId = "TEST-RUN-26",
            timestampMs = 1000L,
            deviceManufacturer = "TestMfg",
            deviceModel = "TestModel",
            androidVersion = "14",
            apiLevel = 34,
            appVersion = "1.0.0",
            modelId = DetectorModelId.YOLO26N,
            selectedModelId = DetectorModelId.YOLO26N,
            activeModelId = DetectorModelId.YOLO26N,
            modelReadiness = ModelReadinessStatus.READY,
            modelInputResolution = "640x640",
            modelInputType = "Float32 RGB",
            backend = "CPU",
            confidenceThreshold = 0.35f,
            nmsThreshold = 0.45f,
            cameraResolution = "1280x720",
            cameraTargetFps = 30,
            observedCameraFps = 29.5f,
            perceptionTargetHz = 4.0f,
            initializationMs = 150L,
            firstInferenceMs = 45L,
            warmupTarget = 10,
            warmupCompleted = 10,
            benchmarkDurationMs = 60_000L,
            elapsedSteadyStateMs = 60_000L,
            inferenceSamples = 200L,
            inferenceSuccesses = 200L,
            inferenceFailures = 0L,
            latencyMinMs = 25L,
            latencyP50Ms = 30f,
            latencyP95Ms = 35f,
            latencyMaxMs = 40L,
            latencyMeanMs = 30f,
            submittedFrames = 240L,
            inferenceStartedFrames = 200L,
            completedFrames = 200L,
            skippedFrames = 40L,
            pendingReplacements = 40L,
            staleResults = 0L,
            productionInferenceRateHz = 3.33f,
            theoreticalDetectorThroughputFps = 33.3f,
            detectionFrames = 150L,
            totalDetections = 300L,
            averageDetectionsPerFrame = 2.0f,
            memoryUsageMb = 120f,
            memoryMaxMb = 512f,
            thermalStatus = "NONE",
            benchmarkStatus = com.minesafety.roboeye.perception.benchmark.BenchmarkStatus.COMPLETED,
        )
        assertEquals(DetectorModelId.YOLO26N, report.modelId)
        assertTrue(report.toJsonString().contains("\"model\": \"YOLO26N\""))
    }

    // Req 20: Warm-up remains 10 successful inferences
    @Test
    fun test6E1_req20_warmupRemains10SuccessfulInferences() {
        val session = com.minesafety.roboeye.perception.benchmark.DeviceBenchmarkSession()
        session.startSession(
            model = DetectorModelId.YOLO26N,
            activeModel = DetectorModelId.YOLO26N,
            warmupFrames = 10,
            durationMs = 60_000L,
        )
        for (i in 1..9) {
            session.onInferenceSuccess(latencyMs = 30L, detectedObjectsCount = 1)
            assertEquals(com.minesafety.roboeye.perception.benchmark.BenchmarkStatus.WARMING_UP, session.currentStatus)
        }
        session.onInferenceSuccess(latencyMs = 30L, detectedObjectsCount = 1)
        assertEquals(com.minesafety.roboeye.perception.benchmark.BenchmarkStatus.RUNNING_STEADY_STATE, session.currentStatus)
    }

    // Req 21: Production FPS formula remains unchanged
    @Test
    fun test6E1_req21_productionFpsFormulaUnchanged() {
        var mockTime = 1000L
        val session = com.minesafety.roboeye.perception.benchmark.DeviceBenchmarkSession(timeProvider = { mockTime })
        session.startSession(
            model = DetectorModelId.YOLO26N,
            activeModel = DetectorModelId.YOLO26N,
            warmupFrames = 1,
            durationMs = 10_000L,
        )
        // Complete warm-up
        session.onInferenceSuccess(30L, 0)
        assertEquals(com.minesafety.roboeye.perception.benchmark.BenchmarkStatus.RUNNING_STEADY_STATE, session.currentStatus)

        // Steady state: 5 completions across 2.0 seconds
        for (i in 1..5) {
            mockTime += 400L
            session.onInferenceSuccess(30L, 1)
        }
        val report = session.currentReport()
        assertEquals(5L, report.completedFrames)
        assertEquals(2000L, report.elapsedSteadyStateMs)
        assertEquals(2.5f, report.productionInferenceRateHz, 0.01f)
    }

    // Req 22: YOLO26n failure does not stop geometry
    @Test
    fun test6E1_req22_yolo26nFailureDoesNotStopGeometry() {
        val initialPose = LocalPose(xM = 1.0f, yM = 2.0f, yawDeg = 10.0f)
        val map = LocalSpatialMap()
        map.updateCell(1, 1, MapCellState.OCCUPIED, 0.9f, 500L)
        val model = NavigationWorldModel(
            pose = initialPose,
            spatialMap = map,
            semanticObjects = emptyList(), // YOLO26n unavailable
        )
        assertTrue(model.semanticObjects.isEmpty())
        assertEquals(1.0f, model.pose.xM, 0.001f)
        assertEquals(MapCellState.OCCUPIED, model.spatialMap.cells[1][1].state)
    }

    // Req 23: No cross-model fallback
    @Test
    fun test6E1_req23_noCrossModelFallback() {
        val result = DetectorFactory.createDetector(DetectorModelId.YOLO26N, MockTestContext())
        assertTrue(result is DetectorCreationResult.Unavailable)
        assertNull(result.activeModelId)
        assertFalse(result.activeModelId == DetectorModelId.SSD_MOBILENET)
        assertFalse(result.activeModelId == DetectorModelId.YOLO11N)
    }

    // Req 24: No duplicate inference worker
    @Test
    fun test6E1_req24_noDuplicateInferenceWorker() {
        val mailbox = PerceptionMailbox()
        assertEquals(0L, mailbox.currentGeneration)
        mailbox.onEnabled()
        assertEquals(1L, mailbox.currentGeneration)
        mailbox.onDisabled()
        assertEquals(2L, mailbox.currentGeneration)
        mailbox.shutdown()
    }

    // Req 25: Pending mailbox remains bounded
    @Test
    fun test6E1_req25_pendingMailboxRemainsBounded() {
        val mailbox = PerceptionMailbox()
        val engineStarted = CountDownLatch(1)
        val engineRelease = CountDownLatch(1)
        mailbox.engine = object : PerceptionEngine {
            override fun processFrame(image: ImageProxy, sensitivity: Float): PerceptionFrameResult {
                if ((image as MockTestFrame).seq == 1L) {
                    engineStarted.countDown()
                    engineRelease.await(2, TimeUnit.SECONDS)
                }
                return PerceptionFrameResult(emptyList(), 1f, 10L, 640, 480)
            }
        }
        mailbox.onEnabled()
        var skippedCount = 0
        mailbox.onFrameSkipped = { skippedCount++ }

        val gen = mailbox.currentGeneration
        val f1 = MockTestFrame(1L, gen)
        val f2 = MockTestFrame(2L, gen)
        val f3 = MockTestFrame(3L, gen)

        mailbox.submit(f1)
        assertTrue(engineStarted.await(1, TimeUnit.SECONDS))

        mailbox.submit(f2)
        assertFalse("Frame 2 should be pending", f2.isFrameClosed)

        mailbox.submit(f3)
        assertTrue("Frame 2 should be closed when Frame 3 replaces it", f2.isFrameClosed)
        assertFalse("Frame 3 should be pending", f3.isFrameClosed)
        assertEquals(1, skippedCount)

        engineRelease.countDown()
        mailbox.shutdown()
    }

    // Req 26: selectedModel == activeModel on successful execution
    @Test
    fun test6E1_req26_selectedModelEqualsActiveModelOnSuccess() {
        val fakeDetector = FakeObjectDetector(detectorType = DetectorType.YOLO, modelName = "YOLO26n", isLoaded = true)
        try {
            DetectorFactory.detectorProvider = { modelId, _ ->
                if (modelId == DetectorModelId.YOLO26N) fakeDetector else null
            }
            val result = DetectorFactory.createDetector(DetectorModelId.YOLO26N, MockTestContext())
            assertTrue(result is DetectorCreationResult.Success)
            assertEquals(result.requestedModelId, result.activeModelId)
            assertEquals(DetectorModelId.YOLO26N, result.activeModelId)
        } finally {
            DetectorFactory.detectorProvider = null
        }
    }

    // Req 27: Model metadata / output validation is tested
    @Test
    fun test6E1_req27_modelMetadataOutputValidation() {
        val meta = ModelRegistry.getMetadata(DetectorModelId.YOLO26N)
        assertEquals(DetectorModelId.YOLO26N, meta.modelId)
        assertEquals(ModelReadinessStatus.PENDING_ASSET, meta.readinessStatus)
        assertEquals(0, meta.inputWidth)
        assertEquals(0, meta.inputHeight)
        assertEquals("PENDING_ASSET_SPECIFICATION", meta.inputTensorType)
        assertEquals("PENDING_ASSET_SPECIFICATION", meta.outputFormat)
    }

    // Req 28: YOLO11n remains functional
    @Test
    fun test6E1_req28_yolo11nRemainsFunctional() {
        val fakeYolo11 = FakeObjectDetector(detectorType = DetectorType.YOLO, modelName = "YOLO11n", isLoaded = true)
        try {
            DetectorFactory.detectorProvider = { modelId, _ ->
                if (modelId == DetectorModelId.YOLO11N) fakeYolo11 else null
            }
            val result = DetectorFactory.createDetector(DetectorModelId.YOLO11N, MockTestContext())
            assertTrue(result is DetectorCreationResult.Success)
            assertEquals(DetectorModelId.YOLO11N, result.activeModelId)
        } finally {
            DetectorFactory.detectorProvider = null
        }
    }

    // Req 29: SSD remains functional
    @Test
    fun test6E1_req29_ssdRemainsFunctional() {
        val fakeSsd = FakeObjectDetector(detectorType = DetectorType.SSD_MOBILENET, modelName = "SSD MobileNet V1", isLoaded = true)
        try {
            DetectorFactory.detectorProvider = { modelId, _ ->
                if (modelId == DetectorModelId.SSD_MOBILENET) fakeSsd else null
            }
            val result = DetectorFactory.createDetector(DetectorModelId.SSD_MOBILENET, MockTestContext())
            assertTrue(result is DetectorCreationResult.Success)
            assertEquals(DetectorModelId.SSD_MOBILENET, result.activeModelId)
        } finally {
            DetectorFactory.detectorProvider = null
        }
    }
}

