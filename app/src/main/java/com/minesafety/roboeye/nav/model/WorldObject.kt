package com.minesafety.roboeye.nav.model

import com.minesafety.roboeye.core.PerceptionObjectType
import com.minesafety.roboeye.core.RelativeMotion

/**
 * Metric certainty classification for monocular distance estimates.
 */
enum class DistanceCertainty {
    /**
     * Estimated using prior bounding-box geometry and camera pitch/height priors.
     * Scale is uncalibrated / heuristic and must not be treated as ground-truth laser metric.
     */
    ESTIMATED_PRIOR,

    /**
     * Distance cannot be estimated or is invalid (e.g. non-positive, NaN, Infinite).
     */
    UNKNOWN,
}

/**
 * Semantic tracked object representation within the NavigationWorldModel.
 *
 * Provides high-level contextual evidence (classification, tracking ID, relative/world geometry).
 * NOTE: Planners in Phase 6B do NOT consume this layer for physical collision avoidance.
 * Authoritative collision avoidance remains strictly geometric (LocalSpatialMap + TTC).
 */
data class WorldObject(
    val trackId: Int,
    val label: String,
    val objectType: PerceptionObjectType,
    val confidence: Float,
    val timestampMs: Long,

    /** Distance from rover camera in meters, or null if distance is unknown. */
    val roverDistanceM: Float? = null,

    /** Bearing angle relative to rover heading in degrees (-180° to +180°, CCW positive, 0° = forward). */
    val roverBearingDeg: Float = 0.0f,

    /** Relative forward position in rover body frame (+X forward, meters), or null if unknown. */
    val roverXM: Float? = null,

    /** Relative lateral position in rover body frame (+Y left, meters), or null if unknown. */
    val roverYM: Float? = null,

    /** Estimated world X coordinate relative to origin, or null if scale/pose is unknown/unreliable. */
    val mapXM: Float? = null,

    /** Estimated world Y coordinate relative to origin, or null if scale/pose is unknown/unreliable. */
    val mapYM: Float? = null,

    /** Estimated physical width in meters, or null if unavailable. */
    val widthM: Float? = null,

    /** Estimated physical height in meters, or null if unavailable. */
    val heightM: Float? = null,

    /** Relative motion state (APPROACHING, RECEDING, STATIONARY, UNKNOWN). */
    val relativeMotion: RelativeMotion = RelativeMotion.UNKNOWN,

    /** Whether the tracker has confirmed this object track. */
    val isConfirmed: Boolean = true,

    /** Whether this object is currently coasting (occluded or missed detection in recent frames). */
    val isCoasting: Boolean = false,

    /** Threat score computed by tracker/spatial heuristic [0.0, 1.0]. */
    val threatScore: Float = 0.0f,

    /** Distance certainty indicator. */
    val distanceCertainty: DistanceCertainty = DistanceCertainty.UNKNOWN,
)
