package com.minesafety.roboeye.localization

import com.minesafety.roboeye.core.ImuReading
import com.minesafety.roboeye.core.model.CameraFrame
import com.minesafety.roboeye.core.model.RobotPose
import com.minesafety.roboeye.mapping.Keyframe
import com.minesafety.roboeye.vision.GeometryTrustLevel
import com.minesafety.roboeye.vision.ImuRotationCompensator
import com.minesafety.roboeye.vision.MotionState
import com.minesafety.roboeye.vision.RansacResult
import com.minesafety.roboeye.vision.RobustFoeEstimator
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow

/**
 * Concrete visual-inertial localizer implementing [LocalizationProvider].
 * Integrates visual motion estimation, complementary IMU yaw fusion,
 * landmark maintenance, and tracking quality state transitions.
 */
class VisualInertialLocalizer(
    val motionEstimator: VisualMotionEstimator = VisualMotionEstimator(),
    val filter: PoseComplementaryFilter = PoseComplementaryFilter(),
    val landmarkManager: LandmarkManager = LandmarkManager(),
    val relocalizer: LocalRelocalizer = LocalRelocalizer(),
) : LocalizationProvider {

    private val _localPoseFlow = MutableStateFlow(LocalPose.ORIGIN)
    val localPoseFlow: StateFlow<LocalPose> = _localPoseFlow.asStateFlow()

    private val _currentPose = MutableStateFlow(
        RobotPose(
            xM = 0.0f,
            yM = 0.0f,
            zM = 0.0f,
            yawDeg = 0.0f,
            confidence = 1.0f,
            frameId = "odom",
            timestampNs = System.nanoTime(),
        )
    )
    override val currentPose: StateFlow<RobotPose> = _currentPose.asStateFlow()

    /**
     * Ingests vision results, IMU reading, and optional frame buffer to update rover pose.
     */
    fun update(
        ransacResult: RansacResult,
        foe: RobustFoeEstimator.FoeResult,
        compensatedFlow: ImuRotationCompensator.CompensatedFlow,
        motionState: MotionState,
        geometryTrust: GeometryTrustLevel,
        imageWidth: Int,
        imageHeight: Int,
        dtSec: Float,
        timestampNs: Long,
        imuReading: ImuReading?,
        isFreshImu: Boolean,
        frame: CameraFrame? = null,
        keyframes: List<Keyframe> = emptyList(),
    ): LocalPose {
        var currentLocal = _localPoseFlow.value

        // 1. Check for relocalization if tracking was lost
        if (currentLocal.trackingState == TrackingQuality.LOST && keyframes.isNotEmpty()) {
            val relocalResult = relocalizer.attemptRelocalization(
                currentPose = currentLocal,
                visibleLandmarks = landmarkManager.activeLandmarks,
                keyframes = keyframes,
            )
            if (relocalResult.isSuccess && relocalResult.targetPose != null) {
                currentLocal = relocalResult.targetPose
            }
        }

        // 2. Derive frame-to-frame body relative visual motion
        val visualMotion = motionEstimator.estimate(
            ransacResult = ransacResult,
            foe = foe,
            compensatedFlow = compensatedFlow,
            motionState = motionState,
            geometryTrust = geometryTrust,
            imageWidth = imageWidth,
            imageHeight = imageHeight,
            dtSec = dtSec,
            timestampNs = timestampNs,
        )

        // 3. Complementary visual-inertial fusion
        val newPose = filter.update(
            prevPose = currentLocal,
            visualMotion = visualMotion,
            imu = imuReading,
            isFreshImu = isFreshImu,
            dtSec = dtSec,
            timestampNs = timestampNs,
        )

        // 4. Update visual landmark tracking
        if (newPose.trackingState != TrackingQuality.LOST) {
            landmarkManager.update(
                inliers = ransacResult.inliers,
                currentPose = newPose,
                frame = frame,
                timestampNs = timestampNs,
            )
        }

        // 5. Update state flows
        _localPoseFlow.value = newPose
        _currentPose.value = RobotPose(
            xM = newPose.xM,
            yM = newPose.yM,
            zM = 0.0f,
            yawDeg = newPose.yawDeg,
            confidence = newPose.confidence,
            frameId = "odom",
            timestampNs = timestampNs,
        )

        return newPose
    }

    override fun resetOrigin() {
        _localPoseFlow.value = LocalPose.ORIGIN
        _currentPose.value = RobotPose(
            xM = 0.0f,
            yM = 0.0f,
            zM = 0.0f,
            yawDeg = 0.0f,
            confidence = 1.0f,
            frameId = "odom",
            timestampNs = System.nanoTime(),
        )
        landmarkManager.clear()
    }
}
