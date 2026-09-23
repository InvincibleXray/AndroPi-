package com.minesafety.roboeye.localization

import com.minesafety.roboeye.vision.FlowVector
import com.minesafety.roboeye.vision.GeometryTrustLevel
import com.minesafety.roboeye.vision.ImuRotationCompensator
import com.minesafety.roboeye.vision.MotionState
import com.minesafety.roboeye.vision.RansacResult
import com.minesafety.roboeye.vision.RobustFoeEstimator
import kotlin.math.hypot

/**
 * Derives frame-to-frame body-relative visual displacement and rotation from
 * RANSAC consensus inliers, Focus-of-Expansion expansion rate, and rotation compensation.
 */
class VisualMotionEstimator(
    private val focalLengthScale: Float = 0.85f,
    private val nominalGroundDistanceM: Float = 1.80f,
    private val minInliersForMotion: Int = 3,
) {

    /**
     * Estimates frame-to-frame relative motion in rover body coordinates.
     */
    fun estimate(
        ransacResult: RansacResult,
        foe: RobustFoeEstimator.FoeResult,
        compensatedFlow: ImuRotationCompensator.CompensatedFlow,
        motionState: MotionState,
        geometryTrust: GeometryTrustLevel,
        imageWidth: Int,
        imageHeight: Int,
        dtSec: Float,
        timestampNs: Long,
    ): VisualRelativeMotion {
        val inliers = ransacResult.inliers
        if (inliers.size < minInliersForMotion || motionState == MotionState.VISUAL_TRACKING_LOST || motionState == MotionState.UNKNOWN) {
            return VisualRelativeMotion(
                timestampNs = timestampNs,
                deltaXM = 0f,
                deltaYM = 0f,
                deltaYawRad = 0f,
                translationConfidence = 0f,
                rotationConfidence = 0f,
                overallConfidence = 0f,
                scaleState = MonocularScaleState.SCALE_UNKNOWN,
                isValid = false,
            )
        }

        val fx = (imageWidth * focalLengthScale).coerceAtLeast(100f)
        val boundedDt = dtSec.coerceIn(0.02f, 0.5f)

        // 1. Visual delta yaw: negative horizontal displacement divided by focal length
        val deltaYawRad = (-(ransacResult.consensusDx / fx)).coerceIn(-0.5f, 0.5f)

        // 2. Translational displacement
        var deltaX = 0.0f
        var deltaY = 0.0f
        var transConf = 0.0f

        when (motionState) {
            MotionState.STATIONARY -> {
                deltaX = 0f
                deltaY = 0f
                transConf = 0.90f
            }
            MotionState.FORWARD_TRANSLATION, MotionState.COMBINED_MOTION -> {
                // Compute mean radial expansion rate away from FOE on compensated flow
                var radialExpansionSum = 0.0f
                var count = 0
                val foeX = foe.x
                val foeY = foe.y

                val motionVectors = compensatedFlow.compensatedVectors
                for (i in motionVectors.indices) {
                    val v = motionVectors[i]
                    val rx = v.prevX - foeX
                    val ry = v.prevY - foeY
                    val dist = hypot(rx.toDouble(), ry.toDouble()).toFloat()
                    if (dist > 15.0f) {
                        val radialDot = (rx * v.dx + ry * v.dy) / dist
                        radialExpansionSum += radialDot / dist
                        count++
                    }
                }

                val avgExpansionRate = if (count > 0) (radialExpansionSum / count) else 0.0f
                val effectiveSpeed = if (avgExpansionRate > 0.02f) {
                    (avgExpansionRate * nominalGroundDistanceM).coerceIn(0.05f, 0.8f)
                } else if (ransacResult.consensusDy > 0.5f) {
                    // Ground plane downward flow velocity: (dy / fx) * (Z / boundedDt)
                    ((ransacResult.consensusDy / fx) * nominalGroundDistanceM / boundedDt).coerceIn(0.05f, 0.8f)
                } else {
                    0.20f // nominal forward crawl
                }
                deltaX = (effectiveSpeed * boundedDt).coerceIn(0.0f, 0.25f)
                deltaY = (-(ransacResult.consensusDx / fx) * 0.15f).coerceIn(-0.10f, 0.10f)

                val foeOrFlowConf = if (foe.confidence > 0.2f) foe.confidence else 0.7f
                transConf = (ransacResult.inlierRatio * foeOrFlowConf).coerceIn(0.1f, 1.0f)
            }
            MotionState.BACKWARD_TRANSLATION -> {
                val effectiveSpeed = if (ransacResult.consensusDy < -0.5f) {
                    ((-ransacResult.consensusDy / fx) * nominalGroundDistanceM / boundedDt).coerceIn(0.05f, 0.5f)
                } else {
                    0.20f
                }
                deltaX = (-effectiveSpeed * boundedDt).coerceIn(-0.20f, 0.0f)
                deltaY = (-(ransacResult.consensusDx / fx) * 0.15f).coerceIn(-0.10f, 0.10f)
                transConf = (ransacResult.inlierRatio * 0.7f).coerceIn(0.1f, 0.8f)
            }
            MotionState.TURNING_LEFT, MotionState.TURNING_RIGHT -> {
                deltaX = 0.02f * boundedDt
                deltaY = 0.0f
                transConf = 0.40f
            }
            else -> {
                deltaX = 0f
                deltaY = 0f
                transConf = 0.1f
            }
        }

        // 3. Monocular scale validity
        val (scaleState, scaleConf) = when (geometryTrust) {
            GeometryTrustLevel.TRUSTED -> {
                if (inliers.size >= 8 && foe.confidence >= 0.5f) {
                    Pair(MonocularScaleState.SCALE_ESTIMATED, (ransacResult.inlierRatio * 0.85f).coerceIn(0.5f, 0.95f))
                } else {
                    Pair(MonocularScaleState.SCALE_UNRELIABLE, 0.40f)
                }
            }
            GeometryTrustLevel.DEGRADED -> Pair(MonocularScaleState.SCALE_UNRELIABLE, 0.25f)
            GeometryTrustLevel.UNTRUSTED -> Pair(MonocularScaleState.SCALE_UNKNOWN, 0.0f)
        }

        val rotConf = (ransacResult.inlierRatio * compensatedFlow.confidenceMultiplier).coerceIn(0.1f, 1.0f)
        val overallConf = (transConf * 0.5f + rotConf * 0.5f).coerceIn(0.0f, 1.0f)

        return VisualRelativeMotion(
            timestampNs = timestampNs,
            deltaXM = deltaX,
            deltaYM = deltaY,
            deltaYawRad = deltaYawRad,
            translationConfidence = transConf,
            rotationConfidence = rotConf,
            overallConfidence = overallConf,
            scaleState = scaleState,
            isValid = true,
        )
    }
}
