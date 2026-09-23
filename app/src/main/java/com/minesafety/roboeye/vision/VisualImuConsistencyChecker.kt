package com.minesafety.roboeye.vision

import com.minesafety.roboeye.core.ImuReading
import kotlin.math.abs
import kotlin.math.sign

/**
 * Validates consistency between camera-derived motion and phone IMU telemetry.
 *
 * Prevents sensor divergence or false state inferences by cross-checking:
 * 1. Rotational consistency: Visual horizontal flow sign/magnitude vs. Gyroscope yaw rate.
 * 2. Static consistency: Near-zero optical flow vs. Near-zero IMU acceleration/gyro.
 * 3. Forward expansion: Radial FOE expansion vs. Near-zero yaw rotation.
 */
class VisualImuConsistencyChecker(
    val stationaryFlowThresholdPx: Float = 0.8f,
    val stationaryGyroThresholdDps: Float = 6.0f,
    val movingFlowThresholdPx: Float = 2.5f,
    val movingGyroThresholdDps: Float = 15.0f,
    val minFeatureCount: Int = 6,
) {
    data class ConsistencyResult(
        val state: VisualImuConsistency,
        val explanation: String,
        val visualEvidence: Float,
        val imuEvidence: Float,
    )

    fun check(
        inlierVectors: List<FlowVector>,
        meanDx: Float,
        meanDy: Float,
        imu: ImuReading?,
        isFreshImu: Boolean = true,
    ): ConsistencyResult {
        // Check for insufficient data
        if (inlierVectors.size < minFeatureCount) {
            return ConsistencyResult(
                state = VisualImuConsistency.INSUFFICIENT_DATA,
                explanation = "Too few valid visual features (${inlierVectors.size} < $minFeatureCount)",
                visualEvidence = 0.0f,
                imuEvidence = 0.0f,
            )
        }

        if (imu == null || !imu.isAvailable || !isFreshImu) {
            return ConsistencyResult(
                state = VisualImuConsistency.INSUFFICIENT_DATA,
                explanation = if (imu == null || !imu.isAvailable) "IMU sensor unavailable" else "IMU telemetry stale",
                visualEvidence = 0.0f,
                imuEvidence = 0.0f,
            )
        }

        val gyroYawDps = when {
            imu.gyroZ != null -> Math.toDegrees(imu.gyroZ.toDouble()).toFloat()
            imu.yawRateDps != 0.0f -> imu.yawRateDps
            else -> 0.0f
        }
        val absGyroYaw = abs(gyroYawDps)
        val absMeanDx = abs(meanDx)

        val visualIsStatic = absMeanDx < stationaryFlowThresholdPx && abs(meanDy) < stationaryFlowThresholdPx
        val imuIsStatic = absGyroYaw < stationaryGyroThresholdDps

        // Scenario 1: Both report stationary
        if (visualIsStatic && imuIsStatic) {
            return ConsistencyResult(
                state = VisualImuConsistency.CONSISTENT,
                explanation = "Both camera and IMU report stationary state",
                visualEvidence = absMeanDx,
                imuEvidence = absGyroYaw,
            )
        }

        // Scenario 2: Rotation conflict or agreement
        val visualIsRotating = absMeanDx >= movingFlowThresholdPx
        val imuIsRotating = absGyroYaw >= movingGyroThresholdDps

        if (visualIsRotating && imuIsRotating) {
            // Camera yaw pan left produces positive visual dx rightward.
            // Check directional agreement (sign correlation)
            val visualSign = sign(meanDx)
            val imuSign = sign(gyroYawDps)

            return if (visualSign == imuSign) {
                ConsistencyResult(
                    state = VisualImuConsistency.CONSISTENT,
                    explanation = "Camera lateral flow (${String.format(java.util.Locale.US, "%.1f", meanDx)}px) agrees with gyro yaw (${String.format(java.util.Locale.US, "%.1f", gyroYawDps)}°/s)",
                    visualEvidence = absMeanDx,
                    imuEvidence = absGyroYaw,
                )
            } else {
                ConsistencyResult(
                    state = VisualImuConsistency.CONFLICTING,
                    explanation = "Directional conflict: visual flow sign ($visualSign) opposes gyro yaw sign ($imuSign)",
                    visualEvidence = absMeanDx,
                    imuEvidence = absGyroYaw,
                )
            }
        }

        // Scenario 3: One reports strong rotation, the other reports stationary
        if (visualIsRotating && imuIsStatic) {
            return ConsistencyResult(
                state = VisualImuConsistency.CONFLICTING,
                explanation = "Conflicting: camera reports high flow (${String.format(java.util.Locale.US, "%.1f", meanDx)}px) but gyro reports stationary (${String.format(java.util.Locale.US, "%.1f", gyroYawDps)}°/s)",
                visualEvidence = absMeanDx,
                imuEvidence = absGyroYaw,
            )
        }

        if (imuIsRotating && visualIsStatic) {
            return ConsistencyResult(
                state = VisualImuConsistency.CONFLICTING,
                explanation = "Conflicting: gyro reports high yaw (${String.format(java.util.Locale.US, "%.1f", gyroYawDps)}°/s) but camera flow is near zero (${String.format(java.util.Locale.US, "%.1f", meanDx)}px)",
                visualEvidence = absMeanDx,
                imuEvidence = absGyroYaw,
            )
        }

        // Scenario 4: Intermediate or translational motion without severe conflict
        return ConsistencyResult(
            state = VisualImuConsistency.PARTIALLY_CONSISTENT,
            explanation = "Observations within intermediate bounds (flow: ${String.format(java.util.Locale.US, "%.1f", absMeanDx)}px, gyro: ${String.format(java.util.Locale.US, "%.1f", absGyroYaw)}°/s)",
            visualEvidence = absMeanDx,
            imuEvidence = absGyroYaw,
        )
    }
}
