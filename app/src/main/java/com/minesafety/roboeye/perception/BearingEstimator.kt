package com.minesafety.roboeye.perception

import com.minesafety.roboeye.core.NormalizedRect
import com.minesafety.roboeye.core.Units
import kotlin.math.atan
import kotlin.math.tan

/**
 * Calculates horizontal angular bearing relative to the camera boresight.
 *
 * Convention:
 *   0.0°     = Directly ahead (aligned with camera boresight)
 *   Negative = Object is to the left of the boresight
 *   Positive = Object is to the right of the boresight
 */
object BearingEstimator {

    /**
     * Estimates horizontal bearing in degrees from a normalized bounding box.
     *
     * @param boundingBox Normalized bounding box in range [0.0, 1.0]
     * @param cameraHfovDeg Camera horizontal field of view in degrees (e.g. 68.0° for Mi 11X main)
     * @return Bearing in degrees, clamped to [-Hfov/2, +Hfov/2]
     */
    fun calculateBearingDeg(boundingBox: NormalizedRect, cameraHfovDeg: Float): Float {
        val centerX = boundingBox.centerX.coerceIn(0.0f, 1.0f)
        // Convert [0.0, 1.0] to optical frame [-1.0, 1.0] where 0.0 is center
        val normalizedX = (centerX - 0.5f) * 2.0f

        val halfFovRad = (cameraHfovDeg * 0.5f) * Units.DEG_TO_RAD
        val tanHalfFov = tan(halfFovRad.toDouble())

        // Optical pinhole projection: tan(bearing) = normalizedX * tan(halfFov)
        val bearingRad = atan(normalizedX * tanHalfFov).toFloat()
        val bearingDeg = bearingRad * Units.RAD_TO_DEG

        return Units.round1(bearingDeg)
    }
}
