package com.minesafety.roboeye.perception

import com.minesafety.roboeye.core.NormalizedRect
import com.minesafety.roboeye.core.PerceptionObjectType
import com.minesafety.roboeye.core.Units
import kotlin.math.atan
import kotlin.math.tan

/**
 * Monocular Visual Distance Estimator.
 *
 * NOTE: This produces an ESTIMATED distance derived from apparent bounding-box geometry,
 * object prior dimensions, and ground-plane declination. It is NEVER a physical RF radar
 * measurement and must be labeled as an estimate across the system.
 */
object DistanceEstimator {

    // Assumed real-world physical heights in meters for mine-site obstacles
    private const val HEIGHT_PERSON_M = 1.70f
    private const val HEIGHT_VEHICLE_M = 2.40f
    private const val HEIGHT_LARGE_OBJECT_M = 3.20f
    private const val HEIGHT_ROAD_OBSTRUCTION_M = 0.65f
    private const val HEIGHT_GENERIC_OBSTACLE_M = 0.90f

    private const val MIN_DISTANCE_M = 0.5f
    private const val MAX_DISTANCE_M = 40.0f

    /**
     * Estimates range in meters from a camera-detected bounding box and device context.
     *
     * @param boundingBox Normalized bounding box in range [0.0, 1.0]
     * @param type Detected object class
     * @param cameraVfovDeg Camera vertical field of view in degrees (e.g. 52.0° for Mi 11X)
     * @param mountHeightM Camera height above ground in meters (e.g. 1.0m)
     * @param imuPitchDeg Device pitch tilt in degrees from IMU (positive = nose down / ground tilted forward)
     * @return Estimated distance in meters, clamped to [0.5, 40.0]
     */
    fun estimateDistanceM(
        boundingBox: NormalizedRect,
        type: PerceptionObjectType,
        cameraVfovDeg: Float = 52.0f,
        mountHeightM: Float = 0.95f,
        imuPitchDeg: Float = 0.0f,
    ): Float {
        val normHeight = boundingBox.height.coerceIn(0.01f, 1.0f)
        val halfVfovRad = (cameraVfovDeg * 0.5f) * Units.DEG_TO_RAD
        val tanHalfVfov = tan(halfVfovRad.toDouble())

        // 1. Pinhole optical size estimation
        val assumedRealHeight = when (type) {
            PerceptionObjectType.PERSON -> HEIGHT_PERSON_M
            PerceptionObjectType.VEHICLE -> HEIGHT_VEHICLE_M
            PerceptionObjectType.LARGE_OBJECT -> HEIGHT_LARGE_OBJECT_M
            PerceptionObjectType.ROAD_OBSTRUCTION -> HEIGHT_ROAD_OBSTRUCTION_M
            PerceptionObjectType.OBSTACLE -> HEIGHT_GENERIC_OBSTACLE_M
        }

        val distFromSize = (assumedRealHeight / (normHeight * 2.0 * tanHalfVfov)).toFloat()

        // 2. Ground-plane projection (from bottom edge of bounding box)
        val bottomY = boundingBox.bottom.coerceIn(0.0f, 1.0f)
        val normYFromCenter = (bottomY - 0.5f) * 2.0f // [-1.0, 1.0], > 0 is below horizon
        val opticalDeclinationRad = atan(normYFromCenter * tanHalfVfov).toFloat()
        val pitchRad = imuPitchDeg * Units.DEG_TO_RAD
        val totalDeclinationRad = opticalDeclinationRad + pitchRad

        val distFromGround = if (totalDeclinationRad > 0.05f) {
            (mountHeightM / tan(totalDeclinationRad.toDouble())).toFloat()
        } else {
            distFromSize
        }

        // 3. Fused estimate
        val estimate = if (distFromGround in MIN_DISTANCE_M..MAX_DISTANCE_M && distFromSize in MIN_DISTANCE_M..MAX_DISTANCE_M) {
            // Weighted average: size is more robust when far, ground plane is more robust when near
            if (distFromSize > 12.0f) {
                0.7f * distFromSize + 0.3f * distFromGround
            } else {
                0.4f * distFromSize + 0.6f * distFromGround
            }
        } else if (distFromSize in MIN_DISTANCE_M..MAX_DISTANCE_M) {
            distFromSize
        } else if (distFromGround in MIN_DISTANCE_M..MAX_DISTANCE_M) {
            distFromGround
        } else {
            distFromSize.coerceIn(MIN_DISTANCE_M, MAX_DISTANCE_M)
        }

        return Units.round1(estimate.coerceIn(MIN_DISTANCE_M, MAX_DISTANCE_M))
    }
}
