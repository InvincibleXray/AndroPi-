package com.minesafety.roboeye.perception

import androidx.camera.core.ImageProxy
import com.minesafety.roboeye.core.RawDetectedObject
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Test
import java.util.concurrent.CountDownLatch
import java.util.concurrent.Executors
import java.util.concurrent.TimeUnit
import java.util.concurrent.atomic.AtomicBoolean
import java.util.concurrent.atomic.AtomicInteger

class PerceptionMailboxTest {

    private class TestFrame(
        val seq: Long,
        val gen: Long,
        val onClosed: (() -> Unit)? = null
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
            onClosed?.invoke()
        }
    }

    private class FakeEngine(
        private val delayMs: Long = 0L,
        private val shouldThrow: Boolean = false,
        private val onProcess: ((ImageProxy) -> Unit)? = null
    ) : PerceptionEngine {
        val processedCount = AtomicInteger(0)
        val processedSequences = mutableListOf<Long>()

        override fun processFrame(image: ImageProxy, sensitivity: Float): PerceptionFrameResult {
            if (delayMs > 0) {
                Thread.sleep(delayMs)
            }
            if (shouldThrow) {
                throw RuntimeException("Simulated detector fault")
            }
            onProcess?.invoke(image)
            val seq = (image as? TestFrame)?.seq ?: 0L
            synchronized(processedSequences) {
                processedSequences.add(seq)
            }
            processedCount.incrementAndGet()
            return PerceptionFrameResult(
                detectedObjects = emptyList(),
                visibilityScore = 0.85f,
                inferenceTimeMs = delayMs,
                frameWidth = image.width,
                frameHeight = image.height,
            )
        }
    }

    @Test
    fun cameraPath_doesNotWaitForDetector() {
        val mailbox = PerceptionMailbox()
        val engineStarted = CountDownLatch(1)
        val engineRelease = CountDownLatch(1)

        val engine = FakeEngine(onProcess = {
            engineStarted.countDown()
            engineRelease.await(2, TimeUnit.SECONDS)
        })

        mailbox.engine = engine
        mailbox.onEnabled()

        val frame = TestFrame(seq = 1L, gen = mailbox.currentGeneration)
        val submitStart = System.currentTimeMillis()
        mailbox.submit(frame)
        val submitDuration = System.currentTimeMillis() - submitStart

        // Submission must be non-blocking (typically < 10ms, definitely < 100ms)
        assertTrue("Submission blocked camera thread for $submitDuration ms", submitDuration < 100)
        assertTrue("Engine did not receive frame", engineStarted.await(1, TimeUnit.SECONDS))

        engineRelease.countDown()
        mailbox.shutdown()
    }

    @Test
    fun boundedMailbox_enforcesMaxOneActiveAndMaxOnePending_replacingOlderPending() {
        val mailbox = PerceptionMailbox()
        val engineStarted = CountDownLatch(1)
        val engineRelease = CountDownLatch(1)
        val frame2Closed = AtomicBoolean(false)
        val frame3Closed = AtomicBoolean(false)
        val resultsDelivered = mutableListOf<Long>()
        val allDone = CountDownLatch(2)

        val engine = FakeEngine(onProcess = { img ->
            if ((img as TestFrame).seq == 1L) {
                engineStarted.countDown()
                engineRelease.await(2, TimeUnit.SECONDS)
            }
        })

        mailbox.engine = engine
        mailbox.onResult = { res ->
            synchronized(resultsDelivered) {
                resultsDelivered.add(res.inferenceTimeMs)
            }
            allDone.countDown()
        }
        mailbox.onEnabled()

        val gen = mailbox.currentGeneration

        // Frame 1 -> becomes active
        val f1 = TestFrame(seq = 1L, gen = gen)
        mailbox.submit(f1)
        assertTrue(engineStarted.await(1, TimeUnit.SECONDS))

        // Frame 2 -> becomes pending
        val f2 = TestFrame(seq = 2L, gen = gen, onClosed = { frame2Closed.set(true) })
        mailbox.submit(f2)
        assertFalse("Frame 2 should still be pending", frame2Closed.get())

        // Frame 3 -> replaces Frame 2 as pending; Frame 2 must be closed immediately
        val f3 = TestFrame(seq = 3L, gen = gen, onClosed = { frame3Closed.set(true) })
        mailbox.submit(f3)
        assertTrue("Frame 2 was not closed when replaced by Frame 3", frame2Closed.get())
        assertFalse("Frame 3 should still be pending", frame3Closed.get())

        // Frame 4 -> replaces Frame 3 as pending; Frame 3 must be closed immediately
        val f4 = TestFrame(seq = 4L, gen = gen)
        mailbox.submit(f4)
        assertTrue("Frame 3 was not closed when replaced by Frame 4", frame3Closed.get())

        // Release engine: active Frame 1 finishes, then pending Frame 4 runs
        engineRelease.countDown()
        assertTrue("Did not complete active and pending executions", allDone.await(2, TimeUnit.SECONDS))

        // Exactly 2 frames should have been processed: #1 (active) and #4 (latest pending)
        assertEquals(2, engine.processedCount.get())
        assertEquals(listOf(1L, 4L), engine.processedSequences)

        mailbox.shutdown()
    }

    @Test
    fun detectorException_isIsolatedAndDoesNotDisruptMailbox() {
        val mailbox = PerceptionMailbox()
        val errorLatch = CountDownLatch(1)
        val nextSuccessLatch = CountDownLatch(1)
        var errorReceived: Throwable? = null

        val throwingEngine = object : PerceptionEngine {
            var calls = 0
            override fun processFrame(image: ImageProxy, sensitivity: Float): PerceptionFrameResult {
                calls++
                if (calls == 1) {
                    throw IllegalStateException("Intentional detector failure")
                }
                return PerceptionFrameResult(
                    detectedObjects = emptyList(),
                    visibilityScore = 0.5f,
                    inferenceTimeMs = 10L,
                    frameWidth = 640,
                    frameHeight = 360,
                )
            }
        }

        mailbox.engine = throwingEngine
        mailbox.onError = { err ->
            errorReceived = err
            errorLatch.countDown()
        }
        mailbox.onResult = {
            nextSuccessLatch.countDown()
        }
        mailbox.onEnabled()

        val gen = mailbox.currentGeneration

        // Submit frame that triggers error
        mailbox.submit(TestFrame(seq = 1L, gen = gen))
        assertTrue("Error was not caught and dispatched to onError", errorLatch.await(1, TimeUnit.SECONDS))
        assertEquals("Intentional detector failure", errorReceived?.message)

        // Submit subsequent frame -> mailbox must continue to operate normally
        mailbox.submit(TestFrame(seq = 2L, gen = gen))
        assertTrue("Subsequent frame was not processed after exception", nextSuccessLatch.await(1, TimeUnit.SECONDS))

        mailbox.shutdown()
    }

    @Test
    fun disabledState_blocksSubmissionsAndDiscardsPendingFrames() {
        val mailbox = PerceptionMailbox()
        val engine = FakeEngine()
        mailbox.engine = engine
        mailbox.onDisabled()

        val frameClosed = AtomicBoolean(false)
        val frame = TestFrame(seq = 1L, gen = mailbox.currentGeneration, onClosed = { frameClosed.set(true) })

        mailbox.submit(frame)

        assertTrue("Frame submitted when disabled must be immediately closed", frameClosed.get())
        assertEquals(0, engine.processedCount.get())

        mailbox.shutdown()
    }

    @Test
    fun disablingWhileRunning_rejectsInFlightResultViaGeneration() {
        val mailbox = PerceptionMailbox()
        val engineStarted = CountDownLatch(1)
        val engineRelease = CountDownLatch(1)
        val resultDelivered = AtomicBoolean(false)

        val engine = FakeEngine(onProcess = {
            engineStarted.countDown()
            engineRelease.await(2, TimeUnit.SECONDS)
        })

        mailbox.engine = engine
        mailbox.onResult = {
            resultDelivered.set(true)
        }
        mailbox.onEnabled()

        // Submit frame 1 while enabled
        mailbox.submit(TestFrame(seq = 1L, gen = mailbox.currentGeneration))
        assertTrue(engineStarted.await(1, TimeUnit.SECONDS))

        // Now user toggles AI Perception OFF while inference is active
        mailbox.onDisabled()

        // Release engine inference
        engineRelease.countDown()
        Thread.sleep(100)

        // The in-flight result must be rejected and not delivered to onResult
        assertFalse("Result of in-flight inference after disable must be rejected", resultDelivered.get())

        mailbox.shutdown()
    }

    @Test
    fun reEnable_createsFreshGenerationAndAcceptsFreshWork() {
        val mailbox = PerceptionMailbox()
        val results = mutableListOf<Long>()
        val resultLatch = CountDownLatch(1)

        val engine = FakeEngine()
        mailbox.engine = engine
        mailbox.onResult = { res ->
            results.add(res.inferenceTimeMs)
            resultLatch.countDown()
        }

        mailbox.onEnabled()
        val gen1 = mailbox.currentGeneration

        mailbox.onDisabled()
        val gen2 = mailbox.currentGeneration
        assertTrue(gen2 > gen1)

        mailbox.onEnabled()
        val gen3 = mailbox.currentGeneration
        assertTrue(gen3 > gen2)

        // Submit fresh frame with gen3
        mailbox.submit(TestFrame(seq = 100L, gen = gen3))
        assertTrue(resultLatch.await(1, TimeUnit.SECONDS))
        assertEquals(1, results.size)

        mailbox.shutdown()
    }

    @Test
    fun outOfOrderResults_areRejected() {
        val mailbox = PerceptionMailbox()
        val acceptedSequences = mutableListOf<Long>()
        val doneLatch = CountDownLatch(2)

        val engine = object : PerceptionEngine {
            override fun processFrame(image: ImageProxy, sensitivity: Float): PerceptionFrameResult {
                val seq = (image as TestFrame).seq
                return PerceptionFrameResult(
                    detectedObjects = listOf(
                        RawDetectedObject(
                            boundingBox = com.minesafety.roboeye.core.NormalizedRect(0f, 0f, 1f, 1f),
                            label = "seq_$seq",
                            type = com.minesafety.roboeye.core.PerceptionObjectType.OBSTACLE,
                            confidence = 0.9f,
                            timestampMs = seq,
                        )
                    ),
                    visibilityScore = 0.9f,
                    inferenceTimeMs = 5L,
                    frameWidth = 640,
                    frameHeight = 360,
                )
            }
        }

        mailbox.engine = engine
        mailbox.onResult = { res ->
            val label = res.detectedObjects.firstOrNull()?.label ?: ""
            val seq = label.substringAfter("seq_").toLongOrNull() ?: 0L
            synchronized(acceptedSequences) {
                acceptedSequences.add(seq)
            }
            doneLatch.countDown()
        }
        mailbox.onEnabled()

        val gen = mailbox.currentGeneration

        // Submit sequence 10 -> processed & accepted
        mailbox.submit(TestFrame(seq = 10L, gen = gen))
        Thread.sleep(50)

        // Submit sequence 5 (stale/out-of-order) -> should be processed by engine but rejected by ordering gate
        mailbox.submit(TestFrame(seq = 5L, gen = gen))
        Thread.sleep(50)

        // Submit sequence 15 -> should be accepted
        mailbox.submit(TestFrame(seq = 15L, gen = gen))
        assertTrue(doneLatch.await(1, TimeUnit.SECONDS))

        // Exactly seq 10 and 15 should be accepted. Seq 5 was rejected!
        assertEquals(listOf(10L, 15L), acceptedSequences)

        mailbox.shutdown()
    }
}
