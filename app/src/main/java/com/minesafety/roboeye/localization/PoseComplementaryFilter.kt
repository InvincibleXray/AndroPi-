package com.minesafety.roboeye.localization

import com.minesafety.roboeye.core.ImuReading
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.sin

/**
 * Lightweight 2D complementary filter fusing visual relative motion with phone IMU
 * gyroscope angular rates, zero-velocity detection, and pose-jump protection.
 */
class PoseComplementaryFilter(
    private val alphaVisualYaw: Float = 0.08f,
    private val maxStepTranslationM: Float = 0.35f,
    private val maxStepRotationDeg: Float = 30.0f,
    private val zuptAccelThresholdMps2: Float = 0.15f,
    private val zuptGyroThresholdDps: Float = 2.0f,
) {

    /**
     * Integrates one visual-inertial step from the previous pose.
     */
    fun update(
        prevPose: LocalPose,
        visualMotion: VisualRelativeMotion,
        imu: ImuReading?,
        isFreshImu: Boolean,
        dtSec: Float,
        timestampNs: Long,
    ): LocalPose {
        val dt = dtSec.coerceIn(0.01f, 0.5f)

        // 1. Gyroscope and Accelerometer ingestion
        val hasUsableImu = imu != null && imu.isAvailable && isFreshImu
        val gyroZdegPerSec = if (hasUsableImu) imu.yawRateDps else 0.0f
        val deltaGyroYawRad = Math.toRadians((gyroZdegPerSec * dt).toDouble()).toFloat()

        // 2. Zero Velocity Detection (ZUPT)
        val isStationaryImu = hasUsableImu &&
                abs(gyroZdegPerSec) < zuptGyroThresholdDps &&
                hypot(imu.axG.toDouble(), imu.ayG.toDouble()) < (zuptAccelThresholdMps2 / 9.81f)

        // 3. Fused Delta Yaw calculation
        val deltaYawRad: Float
        val rotConf: Float

        if (hasUsableImu && visualMotion.isValid) {
            // Nominal fusion: Gyro provides responsive short-term rate, vision eliminates long-term drift
            deltaYawRad = (1.0f - alphaVisualYaw) * deltaGyroYawRad + alphaVisualYaw * visualMotion.deltaYawRad
            rotConf = (0.7f * 0.95f + 0.3f * visualMotion.rotationConfidence).coerceIn(0.1f, 1.0f)
        } else if (hasUsableImu) {
            // Visual tracking degraded: rely solely on gyro
            deltaYawRad = deltaGyroYawRad
            rotConf = 0.65f
        } else if (visualMotion.isValid) {
            // IMU unavailable: rely purely on visual rotation
            deltaYawRad = visualMotion.deltaYawRad
            rotConf = (visualMotion.rotationConfidence * 0.70f).coerceIn(0.1f, 0.85f)
        } else {
            // Neither sensor reliable
            deltaYawRad = 0.0f
            rotConf = 0.0f
        }

        // Clamp delta yaw to protect against wild rotational spikes
        val maxRotRad = Math.toRadians(maxStepRotationDeg.toDouble()).toFloat()
        val clampedDeltaYawRad = deltaYawRad.coerceIn(-maxRotRad, maxRotRad)

        // 4. Update Heading
        val currentYawRad = Math.toRadians(prevPose.yawDeg.toDouble()).toFloat()
        val newYawRad = normalizeRad(currentYawRad + clampedDeltaYawRad)
        val newYawDeg = Math.toDegrees(newYawRad.toDouble()).toFloat()

        // 5. Body translation integration & jump protection
        var bodyDx = visualMotion.deltaXM
        var bodyDy = visualMotion.deltaYM
        var transConf = visualMotion.translationConfidence

        if (isStationaryImu) {
            bodyDx = 0f
            bodyDy = 0f
            transConf = 0.95f
        } else if (!visualMotion.isValid) {
            // Tracking lost: halt translation integration to prevent runaway dead-reckoning drift
            bodyDx = 0f
            bodyDy = 0f
            transConf = 0.0f
        }

        // Clamp translation magnitude
        val transStep = hypot(bodyDx.toDouble(), bodyDy.toDouble()).toFloat()
        var wasClamped = false
        if (transStep > maxStepTranslationM) {
            val scale = maxStepTranslationM / transStep
            bodyDx *= scale
            bodyDy *= scale
            wasClamped = true
        }

        // 6. Transform body translation into world coordinates using mid-point/new heading
        val cosH = cos(newYawRad.toDouble()).toFloat()
        val sinH = sin(newYawRad.toDouble()).toFloat()
        val worldDx = bodyDx * cosH - bodyDy * sinH
        val worldDy = bodyDx * sinH + bodyDy * cosH

        val newXM = prevPose.xM + worldDx
        val newYM = prevPose.yM + worldDy

        // 7. Confidence & Tracking Quality determination
        val visualConf = if (visualMotion.isValid) visualMotion.overallConfidence else 0.0f
        val imuConf = if (hasUsableImu) 0.95f else 0.20f
        var overallConf = (visualConf * 0.5f + imuConf * 0.3f + rotConf * 0.2f).coerceIn(0.0f, 1.0f)
        if (wasClamped) {
            overallConf *= 0.70f
        }

        val trackingState = when {
            !visualMotion.isValid && !hasUsableImu -> TrackingQuality.LOST
            !visualMotion.isValid -> TrackingQuality.DEGRADED
            visualConf < 0.35f || wasClamped -> TrackingQuality.DEGRADED
            else -> TrackingQuality.TRACKING
        }

        return LocalPose(
            timestampNs = timestampNs,
            xM = newXM,
            yM = newYM,
            yawDeg = newYawDeg,
            confidence = overallConf,
            visualConfidence = visualConf,
            imuConfidence = imuConf,
            scaleConfidence = visualMotion.translationConfidence,
            scaleState = visualMotion.scaleState,
            trackingState = trackingState,
        )
    }

    private fun normalizeRad(rad: Float): Float {
        var a = rad
        while (a > Math.PI.toFloat()) a -= (2.0 * Math.PI).toFloat()
        while (a < -Math.PI.toFloat()) a += (2.0 * Math.PI).toFloat()
        return a
    }
}
