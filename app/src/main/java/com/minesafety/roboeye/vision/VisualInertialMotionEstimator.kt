package com.minesafety.roboeye.vision

import com.minesafety.roboeye.core.ImuReading
import kotlin.math.abs
import kotlin.math.sqrt

/**
 * Structured estimate produced by [VisualInertialMotionEstimator].
 */
data class VisualInertialMotionEstimate(
    val motionState: MotionState,
    val consistency: VisualImuConsistency,
    val confidence: Float,
    val translationalFlowPx: Float,
    val rotationalVelocityDps: Float,
    val linearAccelG: Float,
    val inlierCount: Int,
    val isImuAvailable: Boolean,
    val explanation: String,
)

/**
 * Deterministic Visual-Inertial Motion Estimator.
 *
 * Integrates RANSAC-filtered optical flow, FOE radial divergence, phone IMU gyro/accelerometer,
 * and temporal smoothing over recent frames to determine the rover's macroscopic motion state.
 *
 * Implemented specifically for low-end hardware constraints (Redmi 6A):
 * - Zero dynamic object allocations in hot loops
 * - Bounded 3-frame history for noise rejection
 * - Deterministic rule precedence
 */
class VisualInertialMotionEstimator(
    val stationaryFlowLimitPx: Float = 0.8f,
    val stationaryGyroLimitDps: Float = 8.0f,
    val turningGyroThresholdDps: Float = 12.0f,
    val forwardExpansionMinVectors: Int = 8,
    val minTrackingInliers: Int = 6,
) {
    // 5-frame bounded history for temporal stability
    private val history = ArrayDeque<MotionState>(5)
    private var confirmedState: MotionState = MotionState.UNKNOWN
    private var candidateState: MotionState? = null
    private var consecutiveCandidateCount: Int = 0

    @Synchronized
    fun reset() {
        history.clear()
        confirmedState = MotionState.UNKNOWN
        candidateState = null
        consecutiveCandidateCount = 0
    }

    @Synchronized
    fun estimate(
        ransacResult: RansacResult,
        foe: RobustFoeEstimator.FoeResult,
        compensatedFlow: ImuRotationCompensator.CompensatedFlow,
        consistencyResult: VisualImuConsistencyChecker.ConsistencyResult,
        imu: ImuReading?,
        isFreshImu: Boolean,
        imageWidth: Int,
        imageHeight: Int,
    ): VisualInertialMotionEstimate {
        val inlierCount = ransacResult.inliers.size
        val hasImu = imu != null && imu.isAvailable && isFreshImu

        val yawRateDps = when {
            imu != null && imu.yawRateDps != 0.0f -> imu.yawRateDps
            imu?.gyroZ != null -> Math.toDegrees(imu.gyroZ.toDouble()).toFloat()
            else -> 0.0f
        }
        val absYawRate = abs(yawRateDps)

        val linAx = imu?.axG ?: 0.0f
        val linAy = imu?.ayG ?: 0.0f
        val accelMagG = sqrt(linAx * linAx + linAy * linAy)

        val consensusDx = ransacResult.consensusDx
        val consensusDy = ransacResult.consensusDy
        val transMagPx = sqrt(consensusDx * consensusDx + consensusDy * consensusDy)

        val bodyForwardAccelG = imu?.forwardAccelG ?: (-(imu?.azG ?: 0.0f))

        // Forward evidence conditions:
        // 1. Central radial expansion away from FOE
        val isRadialExpansion = foe.isDivergent && inlierCount >= forwardExpansionMinVectors &&
            foe.x in (imageWidth * 0.10f)..(imageWidth * 0.90f) &&
            foe.y in (imageHeight * 0.10f)..(imageHeight * 0.90f)

        // 2. Ground plane downward optical flow (features streaming toward bottom of image)
        val isDownwardGroundFlow = consensusDy >= stationaryFlowLimitPx && absYawRate < stationaryGyroLimitDps

        // 3. Forward linear acceleration confirmation with positive flow
        val isForwardAccelConfirmed = bodyForwardAccelG > 0.08f && consensusDy >= 0.2f

        val isForwardEvidence = isRadialExpansion || isDownwardGroundFlow || isForwardAccelConfirmed

        // Backward evidence conditions:
        // 1. Ground plane upward optical flow (features streaming toward top of image / horizon)
        val isUpwardGroundFlow = consensusDy <= -stationaryFlowLimitPx && absYawRate < stationaryGyroLimitDps

        // 2. Focus of contraction (vectors pointing toward FOE)
        val isRadialContraction = inlierCount >= forwardExpansionMinVectors &&
            foe.isConvergent && foe.confidence >= 0.35f &&
            foe.x in (imageWidth * 0.10f)..(imageWidth * 0.90f) &&
            foe.y in (imageHeight * 0.10f)..(imageHeight * 0.90f)

        // 3. Backward linear acceleration confirmation with negative flow
        val isBackwardAccelConfirmed = bodyForwardAccelG < -0.08f && consensusDy <= -0.2f

        val isBackwardEvidence = (isUpwardGroundFlow || isRadialContraction || isBackwardAccelConfirmed) && !isForwardEvidence

        val rawState: MotionState
        val explanation: String

        // 1. Both sensors completely absent / uninitialized
        if (inlierCount < minTrackingInliers && !hasImu) {
            rawState = MotionState.UNKNOWN
            explanation = "Insufficient data: camera tracking lost and IMU unavailable"
        }
        // 2. Visual tracking lost but IMU is active
        else if (inlierCount < minTrackingInliers) {
            rawState = if (absYawRate > stationaryGyroLimitDps || accelMagG > 0.15f) {
                MotionState.IMU_ONLY
            } else {
                MotionState.VISUAL_TRACKING_LOST
            }
            explanation = "Visual tracking lost (${inlierCount} inliers < $minTrackingInliers); relying on IMU"
        }
        // 3. Stationary check (both sensors report quiet baseline)
        else if (transMagPx < stationaryFlowLimitPx && absYawRate < stationaryGyroLimitDps) {
            rawState = MotionState.STATIONARY
            explanation = "Vehicle stationary (flow: ${fmt(transMagPx)}px, yaw: ${fmt(absYawRate)}°/s)"
        }
        // 4. Turning check (gyro reports high yaw rate)
        else if (absYawRate >= turningGyroThresholdDps) {
            if (isForwardEvidence) {
                rawState = MotionState.COMBINED_MOTION
                explanation = "Combined forward translation and turning (yaw: ${fmt(yawRateDps)}°/s)"
            } else if (yawRateDps > 0) {
                rawState = MotionState.TURNING_LEFT
                explanation = "Turning left (yaw: +${fmt(yawRateDps)}°/s)"
            } else {
                rawState = MotionState.TURNING_RIGHT
                explanation = "Turning right (yaw: -${fmt(absYawRate)}°/s)"
            }
        }
        // 5. Forward Translation
        else if (isForwardEvidence) {
            rawState = MotionState.FORWARD_TRANSLATION
            explanation = when {
                isRadialExpansion -> "Forward translation with radial expansion (FOE: ${foe.x.toInt()}, ${foe.y.toInt()})"
                isDownwardGroundFlow -> "Forward translation with downward optical flow (dy: +${fmt(consensusDy)}px)"
                else -> "Forward translation confirmed by visual-inertial acceleration (+${fmt(bodyForwardAccelG)}g)"
            }
        }
        // 6. Backward Translation (strictly positive backward evidence)
        else if (isBackwardEvidence) {
            rawState = MotionState.BACKWARD_TRANSLATION
            explanation = when {
                isUpwardGroundFlow -> "Backward translation with upward optical flow (dy: ${fmt(consensusDy)}px)"
                isRadialContraction -> "Backward translation with visual flow contraction"
                else -> "Backward translation confirmed by visual-inertial acceleration (${fmt(bodyForwardAccelG)}g)"
            }
        }
        // 7. General motion or conflicting state
        else {
            rawState = if (consistencyResult.state == VisualImuConsistency.CONFLICTING) {
                MotionState.UNKNOWN
            } else {
                MotionState.COMBINED_MOTION
            }
            explanation = "Unclassified or complex motion pattern (flow: ${fmt(transMagPx)}px, yaw: ${fmt(absYawRate)}°/s)"
        }

        // Apply bounded 5-frame temporal stability and state hysteresis
        val smoothedState = updateStateWithHysteresis(rawState)
        val finalExplanation = if (smoothedState != rawState) {
            "$explanation (held by temporal hysteresis: $smoothedState)"
        } else {
            explanation
        }

        // Compute system confidence (0.0 .. 1.0)
        var confidence = (ransacResult.inlierRatio * 0.5f + (inlierCount.toFloat() / 50.0f).coerceAtMost(0.5f))
        confidence *= compensatedFlow.confidenceMultiplier

        if (consistencyResult.state == VisualImuConsistency.CONFLICTING) {
            confidence *= 0.4f
        } else if (consistencyResult.state == VisualImuConsistency.INSUFFICIENT_DATA) {
            confidence *= 0.5f
        }

        val finalConfidence = confidence.coerceIn(0.0f, 1.0f)

        return VisualInertialMotionEstimate(
            motionState = smoothedState,
            consistency = consistencyResult.state,
            confidence = finalConfidence,
            translationalFlowPx = transMagPx,
            rotationalVelocityDps = absYawRate,
            linearAccelG = accelMagG,
            inlierCount = inlierCount,
            isImuAvailable = hasImu,
            explanation = finalExplanation,
        )
    }

    private fun updateStateWithHysteresis(rawState: MotionState): MotionState {
        history.addLast(rawState)
        if (history.size > 5) {
            history.removeFirst()
        }

        val current = confirmedState
        if (current == MotionState.UNKNOWN) {
            confirmedState = rawState
            candidateState = null
            consecutiveCandidateCount = 0
            return rawState
        }

        if (rawState == current) {
            candidateState = null
            consecutiveCandidateCount = 0
            return current
        }

        if (rawState == candidateState) {
            consecutiveCandidateCount++
        } else {
            candidateState = rawState
            consecutiveCandidateCount = 1
        }

        val requiredCount = when {
            // Turning transitions with 2 consecutive frames
            rawState == MotionState.TURNING_LEFT || rawState == MotionState.TURNING_RIGHT -> 2

            // Transition from moving to stationary requires 3 sustained quiet frames (~250-300ms)
            (current == MotionState.FORWARD_TRANSLATION || current == MotionState.BACKWARD_TRANSLATION) &&
                rawState == MotionState.STATIONARY -> 3

            // Direction reversal (Forward <-> Backward) transitions cleanly at 3 consecutive opposing frames
            (current == MotionState.FORWARD_TRANSLATION && rawState == MotionState.BACKWARD_TRANSLATION) ||
                (current == MotionState.BACKWARD_TRANSLATION && rawState == MotionState.FORWARD_TRANSLATION) -> 3

            // Starting translation from stationary requires 2 consecutive frames
            current == MotionState.STATIONARY &&
                (rawState == MotionState.FORWARD_TRANSLATION || rawState == MotionState.BACKWARD_TRANSLATION) -> 2

            // Transient tracking loss / unknown requires 3 consecutive frames before leaving active translation/turning
            rawState == MotionState.VISUAL_TRACKING_LOST || rawState == MotionState.UNKNOWN -> {
                if (current == MotionState.FORWARD_TRANSLATION || current == MotionState.BACKWARD_TRANSLATION ||
                    current == MotionState.TURNING_LEFT || current == MotionState.TURNING_RIGHT) 3 else 2
            }

            else -> 2
        }

        if (consecutiveCandidateCount >= requiredCount) {
            confirmedState = rawState
            candidateState = null
            consecutiveCandidateCount = 0
        }

        return confirmedState
    }

    private fun fmt(v: Float) = String.format(java.util.Locale.US, "%.1f", v)
}
