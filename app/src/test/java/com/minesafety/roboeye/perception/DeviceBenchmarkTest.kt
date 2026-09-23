package com.minesafety.roboeye.perception

import com.minesafety.roboeye.perception.benchmark.BenchmarkStatus
import com.minesafety.roboeye.perception.benchmark.DeviceBenchmarkReport
import com.minesafety.roboeye.perception.benchmark.DeviceBenchmarkSession
import com.minesafety.roboeye.perception.model.DetectorModelId
import com.minesafety.roboeye.perception.model.ModelReadinessStatus
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test
import java.util.concurrent.atomic.AtomicLong

/**
 * Deterministic unit verification suite for Phase 6E Real-Device Multi-Model Benchmark.
 *
 * Covers all 20 required verification criteria defined in Section 26.
 */
class DeviceBenchmarkTest {

    // =========================================================================
    // 1. Warm-up samples are strictly excluded from steady-state statistics
    // =========================================================================
    @Test
    fun test1_warmupSamplesExcludedFromSteadyStateStatistics() {
        var mockTime = 1000L
        val session = DeviceBenchmarkSession(timeProvider = { mockTime })
        session.startSession(
            model = DetectorModelId.YOLO11N,
            activeModel = DetectorModelId.YOLO11N,
            warmupFrames = 3,
            durationMs = 60_000L,
        )

        // Warm-up samples: 500ms, 400ms, 300ms
        session.onInferenceSuccess(500L, 0)
        session.onInferenceSuccess(400L, 0)
        session.onInferenceSuccess(300L, 0)

        // Steady-state samples: 20ms, 30ms, 40ms
        mockTime += 1000L
        session.onInferenceSuccess(20L, 1)
        mockTime += 1000L
        session.onInferenceSuccess(30L, 1)
        mockTime += 1000L
        val report = session.onInferenceSuccess(40L, 1)

        assertEquals(3, report.warmupCompleted)
        assertEquals(3L, report.inferenceSamples)
        assertEquals(3L, report.completedFrames)

        // Crucial verification: warm-up latencies (300, 400, 500) MUST NOT affect steady-state min/max/mean
        assertEquals(20L, report.latencyMinMs)
        assertEquals(40L, report.latencyMaxMs)
        assertEquals(30.0f, report.latencyMeanMs, 0.001f)
    }

    // =========================================================================
    // 2. P50 calculation is deterministic
    // =========================================================================
    @Test
    fun test2_p50CalculationIsDeterministic() {
        var mockTime = 1000L
        val session = DeviceBenchmarkSession(timeProvider = { mockTime })
        session.startSession(
            model = DetectorModelId.YOLO11N,
            activeModel = DetectorModelId.YOLO11N,
            warmupFrames = 1,
            durationMs = 60_000L,
        )
        session.onInferenceSuccess(100L, 0) // warmup

        // 100 steady-state samples: 1..100
        for (i in 1..100) {
            mockTime += 10L
            session.onInferenceSuccess(i.toLong(), 0)
        }

        val report = session.currentReport()
        assertEquals(100L, report.inferenceSamples)
        // 1 to 100: index (99 * 0.5) = 50 -> value 51
        assertTrue("p50 should be ~50ms, was ${report.latencyP50Ms}", report.latencyP50Ms in 50.0f..51.0f)
    }

    // =========================================================================
    // 3. P95 calculation is deterministic
    // =========================================================================
    @Test
    fun test3_p95CalculationIsDeterministic() {
        var mockTime = 1000L
        val session = DeviceBenchmarkSession(timeProvider = { mockTime })
        session.startSession(
            model = DetectorModelId.YOLO11N,
            activeModel = DetectorModelId.YOLO11N,
            warmupFrames = 1,
            durationMs = 60_000L,
        )
        session.onInferenceSuccess(100L, 0) // warmup

        // 100 steady-state samples: 1..100
        for (i in 1..100) {
            mockTime += 10L
            session.onInferenceSuccess(i.toLong(), 0)
        }

        val report = session.currentReport()
        assertEquals(100L, report.inferenceSamples)
        // 1 to 100: index (99 * 0.95) = 94 -> value 95
        assertTrue("p95 should be ~95ms, was ${report.latencyP95Ms}", report.latencyP95Ms in 94.0f..96.0f)
    }

    // =========================================================================
    // 4. Minimum and maximum calculations are deterministic
    // =========================================================================
    @Test
    fun test4_minAndMaxCalculationsAreDeterministic() {
        val session = DeviceBenchmarkSession()
        session.startSession(
            model = DetectorModelId.SSD_MOBILENET,
            activeModel = DetectorModelId.SSD_MOBILENET,
            warmupFrames = 1,
        )
        session.onInferenceSuccess(999L, 0) // warmup

        session.onInferenceSuccess(45L, 0)
        session.onInferenceSuccess(12L, 0)
        session.onInferenceSuccess(88L, 0)
        session.onInferenceSuccess(30L, 0)

        val report = session.currentReport()
        assertEquals(12L, report.latencyMinMs)
        assertEquals(88L, report.latencyMaxMs)
    }

    // =========================================================================
    // 5. Failed inference does not count as successful inference
    // =========================================================================
    @Test
    fun test5_failedInferenceDoesNotCountAsSuccessfulInference() {
        val session = DeviceBenchmarkSession()
        session.startSession(
            model = DetectorModelId.YOLO11N,
            activeModel = DetectorModelId.YOLO11N,
            warmupFrames = 1,
        )
        session.onInferenceSuccess(25L, 0) // warmup

        session.onInferenceSuccess(25L, 0)
        session.onInferenceFailure()
        session.onInferenceSuccess(30L, 0)
        session.onInferenceFailure()

        val report = session.currentReport()
        assertEquals(2L, report.inferenceSuccesses)
        assertEquals(2L, report.inferenceFailures)
        assertEquals(2L, report.inferenceSamples)
    }

    // =========================================================================
    // 6. Skipped frame does not count as completed inference
    // =========================================================================
    @Test
    fun test6_skippedFrameDoesNotCountAsCompletedInference() {
        val session = DeviceBenchmarkSession()
        session.startSession(
            model = DetectorModelId.YOLO11N,
            activeModel = DetectorModelId.YOLO11N,
            warmupFrames = 1,
        )
        session.onInferenceSuccess(20L, 0) // warmup

        session.onInferenceSuccess(25L, 0)
        session.onInferenceSuccess(28L, 0)
        session.onFrameSkipped()
        session.onFrameSkipped()
        session.onFrameSkipped()

        val report = session.currentReport()
        assertEquals(2L, report.completedFrames)
        assertEquals(3L, report.skippedFrames)
        assertFalse(report.completedFrames == report.skippedFrames)
    }

    // =========================================================================
    // 7. Pending-frame replacement is not counted as detector failure
    // =========================================================================
    @Test
    fun test7_pendingFrameReplacementIsNotCountedAsDetectorFailure() {
        val session = DeviceBenchmarkSession()
        session.startSession(
            model = DetectorModelId.YOLO11N,
            activeModel = DetectorModelId.YOLO11N,
            warmupFrames = 1,
        )
        session.onInferenceSuccess(20L, 0)

        // Multiple mailbox frame replacements
        session.onFrameSkipped()
        session.onFrameSkipped()

        val report = session.currentReport()
        assertEquals(0L, report.inferenceFailures)
        assertEquals(2L, report.pendingReplacements)
        assertEquals(2L, report.skippedFrames)
    }

    // =========================================================================
    // 8. Production rate is based on actual completed production results over elapsed steady state
    // =========================================================================
    @Test
    fun test8_productionRateBasedOnActualCompletedResults() {
        var mockTime = 1000L
        val session = DeviceBenchmarkSession(timeProvider = { mockTime })
        session.startSession(
            model = DetectorModelId.YOLO11N,
            activeModel = DetectorModelId.YOLO11N,
            warmupFrames = 1,
            durationMs = 60_000L,
        )
        session.onInferenceSuccess(20L, 0) // warmup ends at T=1000

        // 10 completed frames across 2.0 seconds (T=1000 to T=3000)
        for (i in 1..10) {
            mockTime += 200L
            session.onInferenceSuccess(20L, 0)
        }

        val report = session.currentReport()
        assertEquals(10L, report.completedFrames)
        assertEquals(2000L, report.elapsedSteadyStateMs)
        // Rate = 10 frames / 2.0s = 5.0 Hz
        assertEquals(5.0f, report.productionInferenceRateHz, 0.01f)
    }

    // =========================================================================
    // 9. Production rate is NOT simply 1000 / lastLatencyMs
    // =========================================================================
    @Test
    fun test9_productionRateIsNotSimplyInverseLatency() {
        var mockTime = 1000L
        val session = DeviceBenchmarkSession(timeProvider = { mockTime })
        session.startSession(
            model = DetectorModelId.YOLO11N,
            activeModel = DetectorModelId.YOLO11N,
            warmupFrames = 1,
            durationMs = 60_000L,
        )
        session.onInferenceSuccess(20L, 0) // warmup

        // Inferences take 20ms each, but arrive once per 1000ms (1.0 Hz)
        mockTime += 1000L
        session.onInferenceSuccess(20L, 0)
        mockTime += 1000L
        session.onInferenceSuccess(20L, 0)

        val report = session.currentReport()
        // Theoretical max FPS = 1000 / 20 = 50.0 FPS
        assertEquals(50.0f, report.theoreticalDetectorThroughputFps, 0.01f)
        // Actual production rate = 2 frames / 2.0s = 1.0 Hz
        assertEquals(1.0f, report.productionInferenceRateHz, 0.01f)
        assertFalse(report.productionInferenceRateHz == report.theoreticalDetectorThroughputFps)
    }

    // =========================================================================
    // 10. Model identity remains correct (selectedModelId == activeModelId)
    // =========================================================================
    @Test
    fun test10_modelIdentityRemainsCorrect() {
        val session = DeviceBenchmarkSession()
        val report = session.startSession(
            model = DetectorModelId.SSD_MOBILENET,
            activeModel = DetectorModelId.SSD_MOBILENET,
        )

        assertEquals(DetectorModelId.SSD_MOBILENET, report.selectedModelId)
        assertEquals(DetectorModelId.SSD_MOBILENET, report.activeModelId)
        assertEquals(report.selectedModelId, report.activeModelId)
    }

    // =========================================================================
    // 11. YOLO26N missing asset produces NOT_BENCHMARKED, PENDING_ASSET, ASSET_UNAVAILABLE
    // =========================================================================
    @Test
    fun test11_yolo26nMissingAssetProducesExpectedStatus() {
        val session = DeviceBenchmarkSession()
        val report = session.startSession(
            model = DetectorModelId.YOLO26N,
            activeModel = null,
        )

        assertEquals(BenchmarkStatus.NOT_BENCHMARKED, report.benchmarkStatus)
        assertEquals(ModelReadinessStatus.PENDING_ASSET, report.modelReadiness)
        assertEquals("ASSET_UNAVAILABLE", report.failureReason)
        assertNull(report.activeModelId)
    }

    // =========================================================================
    // 12. Failed model does not activate another model
    // =========================================================================
    @Test
    fun test12_failedModelDoesNotActivateAnotherModel() {
        val session = DeviceBenchmarkSession()
        val report = session.startSession(
            model = DetectorModelId.YOLO11N,
            activeModel = null, // failed
        )

        assertEquals(BenchmarkStatus.FAILED, report.benchmarkStatus)
        assertEquals(DetectorModelId.YOLO11N, report.selectedModelId)
        assertNull(report.activeModelId)
        assertFalse(report.activeModelId == DetectorModelId.SSD_MOBILENET)
    }

    // =========================================================================
    // 13. Benchmark reset isolates one model from another
    // =========================================================================
    @Test
    fun test13_benchmarkResetIsolatesOneModelFromAnother() {
        val session = DeviceBenchmarkSession()
        session.startSession(
            model = DetectorModelId.YOLO11N,
            activeModel = DetectorModelId.YOLO11N,
            warmupFrames = 1,
        )
        session.onInferenceSuccess(10L, 0)
        session.onInferenceSuccess(20L, 2)
        session.onInferenceSuccess(30L, 3)

        // Reset clears previous run completely
        session.reset()
        val idleReport = session.currentReport()
        assertEquals(BenchmarkStatus.IDLE, idleReport.benchmarkStatus)
        assertEquals(0L, idleReport.inferenceSamples)
        assertEquals(0L, idleReport.totalDetections)

        // Next model run is completely isolated
        session.startSession(
            model = DetectorModelId.SSD_MOBILENET,
            activeModel = DetectorModelId.SSD_MOBILENET,
            warmupFrames = 1,
        )
        session.onInferenceSuccess(50L, 0)
        val report = session.onInferenceSuccess(60L, 1)
        assertEquals(1L, report.inferenceSamples)
        assertEquals(60L, report.latencyMinMs)
        assertEquals(60L, report.latencyMaxMs)
    }

    // =========================================================================
    // 14. NaN is sanitized
    // =========================================================================
    @Test
    fun test14_nanIsSanitized() {
        assertEquals(0.0f, DeviceBenchmarkSession.sanitize(Float.NaN), 0.0f)
        assertFalse(DeviceBenchmarkSession.sanitize(Float.NaN).isNaN())
    }

    // =========================================================================
    // 15. Infinity is sanitized
    // =========================================================================
    @Test
    fun test15_infinityIsSanitized() {
        assertEquals(0.0f, DeviceBenchmarkSession.sanitize(Float.POSITIVE_INFINITY), 0.0f)
        assertEquals(0.0f, DeviceBenchmarkSession.sanitize(Float.NEGATIVE_INFINITY), 0.0f)
        assertFalse(DeviceBenchmarkSession.sanitize(Float.POSITIVE_INFINITY).isInfinite())
    }

    // =========================================================================
    // 16. Zero-sample benchmark is safe (no divide-by-zero)
    // =========================================================================
    @Test
    fun test16_zeroSampleBenchmarkIsSafe() {
        val session = DeviceBenchmarkSession()
        val report = session.startSession(
            model = DetectorModelId.YOLO11N,
            activeModel = DetectorModelId.YOLO11N,
        )

        assertEquals(0L, report.inferenceSamples)
        assertEquals(0.0f, report.latencyMeanMs, 0.0f)
        assertEquals(0.0f, report.latencyP50Ms, 0.0f)
        assertEquals(0.0f, report.latencyP95Ms, 0.0f)
        assertEquals(0.0f, report.productionInferenceRateHz, 0.0f)
        assertFalse(report.productionInferenceRateHz.isNaN())
    }

    // =========================================================================
    // 17. Insufficient warm-up samples are handled safely
    // =========================================================================
    @Test
    fun test17_insufficientWarmupSamplesHandledSafely() {
        val session = DeviceBenchmarkSession()
        session.startSession(
            model = DetectorModelId.YOLO11N,
            activeModel = DetectorModelId.YOLO11N,
            warmupFrames = 10,
        )
        // Only 3 warmups before failure
        session.onInferenceSuccess(20L, 0)
        session.onInferenceSuccess(20L, 0)
        session.onInferenceSuccess(20L, 0)
        val report = session.onInferenceFailure("Camera stream terminated")

        assertEquals(BenchmarkStatus.FAILED, report.benchmarkStatus)
        assertEquals(3, report.warmupCompleted)
        assertEquals(0L, report.inferenceSamples) // steady state never entered
        assertTrue(report.failureReason!!.contains("WARMUP_FAILED"))
    }

    // =========================================================================
    // 18. Initialization failure is distinct from runtime failure
    // =========================================================================
    @Test
    fun test18_initializationFailureIsDistinctFromRuntimeFailure() {
        val sessionA = DeviceBenchmarkSession()
        val initFailReport = sessionA.startSession(
            model = DetectorModelId.YOLO11N,
            activeModel = null,
        )
        assertEquals(BenchmarkStatus.FAILED, initFailReport.benchmarkStatus)
        assertEquals("INITIALIZATION_FAILED", initFailReport.failureReason)
        assertEquals(0L, initFailReport.inferenceFailures)

        val sessionB = DeviceBenchmarkSession()
        sessionB.startSession(
            model = DetectorModelId.YOLO11N,
            activeModel = DetectorModelId.YOLO11N,
            warmupFrames = 1,
        )
        sessionB.onInferenceSuccess(20L, 0) // warmup
        sessionB.onInferenceFailure("GPU Out of Memory")
        val runtimeFailReport = sessionB.currentReport()
        assertEquals(1L, runtimeFailReport.inferenceFailures)
    }

    // =========================================================================
    // 19. Selected model remains unchanged when active model becomes null
    // =========================================================================
    @Test
    fun test19_selectedModelRemainsUnchangedWhenActiveModelBecomesNull() {
        val session = DeviceBenchmarkSession()
        val report = session.startSession(
            model = DetectorModelId.SSD_MOBILENET,
            activeModel = null, // Unavailable
        )

        assertEquals(DetectorModelId.SSD_MOBILENET, report.selectedModelId)
        assertNull(report.activeModelId)
    }

    // =========================================================================
    // 20. Completed inference count cannot exceed inference-start count
    // =========================================================================
    @Test
    fun test20_completedInferenceCountCannotExceedInferenceStartCount() {
        val session = DeviceBenchmarkSession()
        session.startSession(
            model = DetectorModelId.YOLO11N,
            activeModel = DetectorModelId.YOLO11N,
            warmupFrames = 1,
        )
        session.onInferenceStarted()
        session.onInferenceSuccess(20L, 0) // warmup

        session.onInferenceStarted()
        session.onInferenceSuccess(25L, 0)
        session.onInferenceStarted()
        session.onInferenceSuccess(28L, 0)

        val report = session.currentReport()
        assertTrue(report.completedFrames <= report.inferenceStartedFrames)
    }

    // =========================================================================
    // 21. JSON Serialization produces comprehensive structured output
    // =========================================================================
    @Test
    fun test21_jsonSerializationMatchesSchema() {
        val session = DeviceBenchmarkSession()
        session.startSession(
            model = DetectorModelId.YOLO11N,
            activeModel = DetectorModelId.YOLO11N,
            warmupFrames = 1,
        )
        session.onInferenceSuccess(20L, 0)
        session.onInferenceSuccess(25L, 2)
        val report = session.currentReport()
        val json = report.toJsonString()

        assertNotNull(json)
        assertTrue(json.contains("\"identity\""))
        assertTrue(json.contains("\"model\""))
        assertTrue(json.contains("\"camera\""))
        assertTrue(json.contains("\"timing\""))
        assertTrue(json.contains("\"latency\""))
        assertTrue(json.contains("\"production\""))
        assertTrue(json.contains("\"detection\""))
        assertTrue(json.contains("\"system\""))
        assertTrue(json.contains("\"status\""))
        assertTrue(json.contains("\"runId\""))
    }
}
