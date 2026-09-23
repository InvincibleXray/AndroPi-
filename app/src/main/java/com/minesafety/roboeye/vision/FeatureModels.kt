package com.minesafety.roboeye.vision

/**
 * 2D visual feature point on the camera image plane.
 */
data class FeaturePoint(
    val x: Float,
    val y: Float,
    val response: Float = 0.0f,
    val id: Long = -1L,
)
