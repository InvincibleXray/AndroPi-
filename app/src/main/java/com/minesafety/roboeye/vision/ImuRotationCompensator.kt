package com.minesafety.roboeye.vision

import com.minesafety.roboeye.core.ImuReading
import kotlin.math.abs
import kotlin.math.sqrt

/**
 * Disentangles camera ego-rotation from scene translation in sparse optical flow.
 *
 * Observed optical flow is the sum of translational flow and rotational flow:
 *   v_obs = v_trans + v_rot
 *
 * Using phone gyroscope telemetry (pitch rate wx, roll rate wy, yaw rate wz):
 *   v_trans ≈ v_obs - v_rot
 *
 * In addition to per-vector rotational subtraction, this compensator computes a
 * confidence damping factor that prevents turning and pivot maneuvers from triggering
 * false forward collision alerts.
 */
class ImuRotationCompensator(
    val nominalFocalLengthPx: Float = 550.0f,
    val rotationThresholdDps: Float = 15.0f,
    val severeRotationThresholdDps: Float = 45.0f,
) {
    data class CompensatedFlow(
        val compensatedVectors: List<FlowVector>,
        val estimatedRotationalDx: Float,
        val estimatedRotationalDy: Float,
        val yawRateDps: Float,
        val pitchRateDps: Float,
        val confidenceMultiplier: Float,
        val isRotationDominant: Boolean,
    )

    fun compensate(
        vectors: List<FlowVector>,
        imu: ImuReading?,
        dtSec: Float,
        imageWidth: Int,
        imageHeight: Int,
    ): CompensatedFlow {
        if (vectors.isEmpty() || imu == null || !imu.isAvailable) {
            return CompensatedFlow(
                compensatedVectors = vectors,
                estimatedRotationalDx = 0.0f,
                estimatedRotationalDy = 0.0f,
                yawRateDps = 0.0f,
                pitchRateDps = 0.0f,
                confidenceMultiplier = 1.0f,
                isRotationDominant = false,
            )
        }

        // Extract angular velocities in radians per second
        // Phone mounted in portrait:
        // Gyro Z (wz) is in-plane roll or yaw depending on mount; imu.yawRateDps holds calibrated vehicle yaw.
        val yawRateRad = if (imu.gyroZ != null) {
            imu.gyroZ
        } else {
            Math.toRadians(imu.yawRateDps.toDouble()).toFloat()
        }
        val pitchRateRad = imu.gyroX ?: 0.0f
        val rollRateRad = imu.gyroY ?: 0.0f

        val yawRateDps = abs(Math.toDegrees(yawRateRad.toDouble()).toFloat())
        val pitchRateDps = abs(Math.toDegrees(pitchRateRad.toDouble()).toFloat())

        // Calculate confidence damping factor
        val confidenceMultiplier = when {
            yawRateDps <= rotationThresholdDps -> 1.0f
            yawRateDps >= severeRotationThresholdDps -> 0.05f
            else -> {
                val excess = yawRateDps - rotationThresholdDps
                val range = severeRotationThresholdDps - rotationThresholdDps
                (1.0f - 0.95f * (excess / range)).coerceIn(0.05f, 1.0f)
            }
        }
        val isRotationDominant = confidenceMultiplier < 0.4f

        val cx = imageWidth / 2.0f
        val cy = imageHeight / 2.0f
        val f = nominalFocalLengthPx
        val dt = dtSec.coerceIn(0.01f, 0.5f)

        // Bulk rotational shifts
        // Yaw (pan): camera pan left (wz > 0) causes scene to move right (dx > 0)
        // dx_rot = f * wz * dt
        val bulkRotDx = f * yawRateRad * dt
        // Pitch (tilt): camera tilting down (wx > 0) causes scene to move up (dy < 0)
        val bulkRotDy = -f * pitchRateRad * dt

        val compensated = ArrayList<FlowVector>(vectors.size)
        for (v in vectors) {
            if (!v.isValid) {
                compensated.add(v)
                continue
            }

            // In-plane tangential rotation contribution from roll (around optical axis)
            val relX = v.currX - cx
            val relY = v.currY - cy
            val rollRotDx = -relY * rollRateRad * dt
            val rollRotDy = relX * rollRateRad * dt

            val totalRotDx = bulkRotDx + rollRotDx
            val totalRotDy = bulkRotDy + rollRotDy

            val cDx = v.dx - totalRotDx
            val cDy = v.dy - totalRotDy

            compensated.add(
                v.copy(
                    currX = v.prevX + cDx,
                    currY = v.prevY + cDy,
                    confidence = v.confidence * confidenceMultiplier,
                )
            )
        }

        return CompensatedFlow(
            compensatedVectors = compensated,
            estimatedRotationalDx = bulkRotDx,
            estimatedRotationalDy = bulkRotDy,
            yawRateDps = yawRateDps,
            pitchRateDps = pitchRateDps,
            confidenceMultiplier = confidenceMultiplier,
            isRotationDominant = isRotationDominant,
        )
    }
}
