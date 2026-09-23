package com.minesafety.roboeye.localization

import com.minesafety.roboeye.mapping.Keyframe
import kotlin.math.hypot

/**
 * Outcome of a local relocalization attempt against stored keyframes.
 */
data class RelocalizationResult(
    val isSuccess: Boolean,
    val targetPose: LocalPose? = null,
    val matchedKeyframeId: Int? = null,
    val matchedLandmarksCount: Int = 0,
    val matchConfidence: Float = 0.0f,
)

/**
 * Lightweight local relocalizer that matches current visible visual landmark descriptors
 * against stored keyframes using bitwise Hamming distance and geometric consensus.
 */
class LocalRelocalizer(
    private val maxHammingDistance: Int = 42, // ~32% bit tolerance out of 128 bits
    private val minMatchingLandmarks: Int = 4,
    private val maxCorrectionDistanceM: Float = 0.60f,
) {

    /**
     * Attempts to relocalize the rover pose against a list of keyframes using visible landmarks.
     */
    fun attemptRelocalization(
        currentPose: LocalPose,
        visibleLandmarks: List<Landmark>,
        keyframes: List<Keyframe>,
    ): RelocalizationResult {
        if (visibleLandmarks.size < minMatchingLandmarks || keyframes.isEmpty()) {
            return RelocalizationResult(isSuccess = false)
        }

        var bestMatchCount = 0
        var bestKeyframe: Keyframe? = null

        for (i in keyframes.indices) {
            val kf = keyframes[i]
            var matches = 0

            for (j in visibleLandmarks.indices) {
                val currentLm = visibleLandmarks[j]
                for (k in kf.landmarks.indices) {
                    val kfLm = kf.landmarks[k]
                    val dist = hammingDistance(currentLm.descriptor, kfLm.descriptor)
                    if (dist <= maxHammingDistance) {
                        matches++
                        break
                    }
                }
            }

            if (matches >= minMatchingLandmarks && matches > bestMatchCount) {
                bestMatchCount = matches
                bestKeyframe = kf
            }
        }

        val matchedKf = bestKeyframe ?: return RelocalizationResult(isSuccess = false)

        // Calculate bounded correction towards matched keyframe pose
        val kfPose = matchedKf.pose
        val dx = kfPose.xM - currentPose.xM
        val dy = kfPose.yM - currentPose.yM
        val distM = hypot(dx.toDouble(), dy.toDouble()).toFloat()

        val scale = if (distM > maxCorrectionDistanceM && distM > 0.001f) {
            maxCorrectionDistanceM / distM
        } else {
            1.0f
        }

        val correctedX = currentPose.xM + dx * scale
        val correctedY = currentPose.yM + dy * scale
        val matchConfidence = (bestMatchCount.toFloat() / visibleLandmarks.size.coerceAtLeast(1))
            .coerceIn(0.5f, 0.95f)

        val targetPose = currentPose.copy(
            xM = correctedX,
            yM = correctedY,
            yawDeg = kfPose.yawDeg,
            confidence = matchConfidence,
            trackingState = TrackingQuality.TRACKING,
        )

        return RelocalizationResult(
            isSuccess = true,
            targetPose = targetPose,
            matchedKeyframeId = matchedKf.id,
            matchedLandmarksCount = bestMatchCount,
            matchConfidence = matchConfidence,
        )
    }

    /**
     * Bitwise Hamming distance between two 16-byte descriptors.
     */
    fun hammingDistance(d1: ByteArray, d2: ByteArray): Int {
        if (d1.size != 16 || d2.size != 16) return 128
        var dist = 0
        for (i in 0 until 16) {
            val xor = (d1[i].toInt() xor d2[i].toInt()) and 0xFF
            dist += Integer.bitCount(xor)
        }
        return dist
    }
}
