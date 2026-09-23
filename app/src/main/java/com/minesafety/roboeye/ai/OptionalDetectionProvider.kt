package com.minesafety.roboeye.ai

import com.minesafety.roboeye.core.RawDetectedObject
import com.minesafety.roboeye.core.model.CameraFrame

/**
 * Isolated boundary for strictly optional neural AI detection (YOLO / SSD MobileNet).
 *
 * Guaranteed Architecture:
 * - Core Rover Brain autonomous navigation DOES NOT depend on this interface.
 * - By default, this provider is DISABLED (especially on low-end Redmi 6A hardware).
 * - Only enabled on-demand when semantic object identification is explicitly requested.
 */
interface OptionalDetectionProvider : AutoCloseable {
    /** Whether an AI neural model is loaded and ready. */
    val isModelLoaded: Boolean

    /** Model descriptor label. */
    val modelName: String

    /** Performs semantic object detection on a camera frame. */
    fun detectObjects(frame: CameraFrame, confidenceThreshold: Float = 0.35f): List<RawDetectedObject>
}

/**
 * Default no-op implementation when AI is disabled.
 * Consumes zero memory, zero GPU/CPU cycles, and loads no TFLite models into memory.
 */
class NoOpDetectionProvider : OptionalDetectionProvider {
    override val isModelLoaded: Boolean = false
    override val modelName: String = "AI Detection Disabled (Baseline Navigation Active)"

    override fun detectObjects(frame: CameraFrame, confidenceThreshold: Float): List<RawDetectedObject> {
        return emptyList()
    }

    override fun close() {
        // No-op
    }
}
