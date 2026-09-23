package com.minesafety.roboeye.localization

import com.minesafety.roboeye.core.model.CameraFrame
import com.minesafety.roboeye.vision.FlowVector
import kotlin.math.cos
import kotlin.math.sin

/**
 * Compact visual landmark with local 2D map position and a 16-byte binary patch descriptor.
 */
data class Landmark(
    val id: Int,
    var mapXM: Float,
    var mapYM: Float,
    var imageX: Float,
    var imageY: Float,
    val descriptor: ByteArray,
    var observationCount: Int = 1,
    var quality: Float = 1.0f,
    var lastSeenTimestampNs: Long = 0L,
) {
    override fun equals(other: Any?): Boolean {
        if (this === other) return true
        if (javaClass != other?.javaClass) return false
        other as Landmark
        return id == other.id
    }

    override fun hashCode(): Int = id
}

/**
 * Manages active visual landmarks, computes 16-byte binary patch signatures,
 * and maintains a bounded pool of up to [maxCapacity] landmarks.
 */
class LandmarkManager(
    private val maxCapacity: Int = 40,
    private val nominalGroundDistanceM: Float = 2.0f,
) {
    private val landmarks = mutableListOf<Landmark>()
    private var nextLandmarkId = 1

    val activeLandmarks: List<Landmark> get() = landmarks

    /**
     * Updates landmarks from current tracking inliers and current rover pose.
     */
    fun update(
        inliers: List<FlowVector>,
        currentPose: LocalPose,
        frame: CameraFrame?,
        timestampNs: Long,
    ): List<Landmark> {
        if (inliers.isEmpty()) return landmarks

        val cosYaw = cos(Math.toRadians(currentPose.yawDeg.toDouble())).toFloat()
        val sinYaw = sin(Math.toRadians(currentPose.yawDeg.toDouble())).toFloat()

        // Match existing landmarks by spatial proximity in image space
        val matchedIds = mutableSetOf<Int>()
        for (i in inliers.indices) {
            val vec = inliers[i]
            var matched: Landmark? = null
            for (j in landmarks.indices) {
                val lm = landmarks[j]
                val dx = lm.imageX - vec.prevX
                val dy = lm.imageY - vec.prevY
                if (dx * dx + dy * dy < 25.0f && !matchedIds.contains(lm.id)) {
                    matched = lm
                    break
                }
            }

            if (matched != null) {
                matched.imageX = vec.currX
                matched.imageY = vec.currY
                matched.observationCount++
                matched.lastSeenTimestampNs = timestampNs
                matched.quality = (matched.quality * 0.9f + vec.confidence * 0.1f).coerceIn(0f, 1f)
                matchedIds.add(matched.id)
            } else if (landmarks.size < maxCapacity && frame != null) {
                val desc = extractDescriptor(frame, vec.currX.toInt(), vec.currY.toInt())
                val bodyX = nominalGroundDistanceM
                val bodyY = -((vec.currX - frame.width * 0.5f) / (frame.width * 0.85f)) * nominalGroundDistanceM
                val mapX = currentPose.xM + bodyX * cosYaw - bodyY * sinYaw
                val mapY = currentPose.yM + bodyX * sinYaw + bodyY * cosYaw

                landmarks.add(
                    Landmark(
                        id = nextLandmarkId++,
                        mapXM = mapX,
                        mapYM = mapY,
                        imageX = vec.currX,
                        imageY = vec.currY,
                        descriptor = desc,
                        observationCount = 1,
                        quality = vec.confidence,
                        lastSeenTimestampNs = timestampNs,
                    )
                )
            }
        }

        pruneIfNeeded()
        return landmarks
    }

    /**
     * Extracts a fast, deterministic 16-byte (128-bit) binary descriptor from the Y-channel buffer.
     */
    fun extractDescriptor(frame: CameraFrame, centerX: Int, centerY: Int): ByteArray {
        val desc = ByteArray(16)
        val yBuf = frame.yBuffer ?: return desc
        val w = frame.width
        val h = frame.height
        val stride = frame.yRowStride

        val offsetsX = intArrayOf(-6, -4, -2, 0, 2, 4, 6, -5, -3, 0, 3, 5, -4, 0, 4, -2)
        val offsetsY = intArrayOf(-6, -2, 2, 6, -4, 0, 4, -5, 1, -3, 3, 5, 0, -4, 4, 2)

        var byteIdx = 0
        var bitIdx = 0
        var currentByte = 0

        for (i in 0 until 128) {
            val idx1 = i % 16
            val idx2 = (i * 7 + 3) % 16

            val x1 = (centerX + offsetsX[idx1]).coerceIn(0, w - 1)
            val y1 = (centerY + offsetsY[idx1]).coerceIn(0, h - 1)
            val x2 = (centerX + offsetsX[idx2]).coerceIn(0, w - 1)
            val y2 = (centerY + offsetsY[idx2]).coerceIn(0, h - 1)

            val p1 = yBuf.get(y1 * stride + x1).toInt() and 0xFF
            val p2 = yBuf.get(y2 * stride + x2).toInt() and 0xFF

            if (p1 > p2) {
                currentByte = currentByte or (1 shl bitIdx)
            }
            bitIdx++
            if (bitIdx == 8) {
                desc[byteIdx++] = currentByte.toByte()
                currentByte = 0
                bitIdx = 0
            }
        }
        return desc
    }

    /**
     * Enforces hard upper bound by pruning least-observed and oldest landmarks.
     */
    fun pruneIfNeeded() {
        if (landmarks.size > maxCapacity) {
            landmarks.sortBy { it.observationCount }
            while (landmarks.size > maxCapacity) {
                landmarks.removeAt(0)
            }
        }
    }

    fun clear() {
        landmarks.clear()
        nextLandmarkId = 1
    }
}
