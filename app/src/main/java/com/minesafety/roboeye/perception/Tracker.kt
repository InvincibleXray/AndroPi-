package com.minesafety.roboeye.perception

import com.minesafety.roboeye.core.ImuReading
import com.minesafety.roboeye.core.RawDetectedObject
import com.minesafety.roboeye.core.TrackedObject

/**
 * Supported object tracker implementations in Robo Eye.
 */
enum class TrackerType(val label: String) {
    BYTE_TRACK("ByteTrack (IMU-CMC)"),
}

/**
 * Interface for multi-object tracking in the perception pipeline.
 */
interface Tracker {
    val trackerType: TrackerType

    fun update(
        detections: List<RawDetectedObject>,
        cameraHfovDeg: Float,
        cameraVfovDeg: Float,
        cameraHeightM: Float,
        imuPitchDeg: Float,
        imuReading: ImuReading = ImuReading(),
        nowMs: Long = System.currentTimeMillis(),
    ): List<TrackedObject>

    fun reset()
}
