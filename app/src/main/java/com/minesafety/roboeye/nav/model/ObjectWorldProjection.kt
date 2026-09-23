package com.minesafety.roboeye.nav.model

import com.minesafety.roboeye.core.NormalizedRect
import com.minesafety.roboeye.core.TrackedObject
import com.minesafety.roboeye.core.Units
import com.minesafety.roboeye.localization.LocalPose
import com.minesafety.roboeye.localization.MonocularScaleState
import com.minesafety.roboeye.localization.TrackingQuality
import kotlin.math.cos
import kotlin.math.sin

/**
 * Pure, stateless projection utility converting perception [TrackedObject]s
 * into semantic [WorldObject] representations for the [NavigationWorldModel].
 *
 * Adheres strictly to the rover coordinate and scale conventions:
 * 1. Image Ground Contact: Uses bottom-center `(centerX, bottom)` of normalized bounding box.
 * 2. Camera to Rover Transform:
 *    Camera optical bearing (0° ahead, -left, +right) transforms to Rover body heading
 *    (+X forward, +Y left, 0° forward, +left, -right): `roverBearingDeg = -bearingDeg`.
 * 3. Rover Relative Coordinates:
 *    `roverXM = distance * cos(roverBearingRad)` (+X forward)
 *    `roverYM = distance * sin(roverBearingRad)` (+Y left)
 * 4. Scale & Distance Certainty:
 *    - CASE A: Valid estimated distance + `pose.scaleState == SCALE_ESTIMATED` + tracking valid ->
 *              computes SE(2) map coordinates `(mapXM, mapYM)` with `ESTIMATED_PRIOR`.
 *    - CASE B: Valid estimated distance + unknown/unreliable scale ->
 *              retains rover coordinates `(roverXM, roverYM)` but leaves `mapXM = null`, `mapYM = null`
 *              with `ESTIMATED_PRIOR`.
 *    - CASE C: Invalid or unknown distance (<= 0, NaN, Inf) ->
 *              leaves all metric coordinates null with `DistanceCertainty.UNKNOWN`.
 * 5. Lifecycle & Staleness:
 *    - Only confirmed tracks ([TrackedObject.isConfirmed]) are projected.
 *    - Coasting tracks ([TrackedObject.isCoasting]) are retained while within [MAX_COAST_AGE_MS].
 *    - Stale or removed tracks are dropped.
 */
object ObjectWorldProjection {

    const val MAX_COAST_AGE_MS: Long = 1000L

    /**
     * Calculates the normalized image coordinate of the approximate ground contact point.
     * Convention: Bottom-center of the normalized 2D bounding box `(centerX, bottom)`.
     */
    fun calculateGroundContact(box: NormalizedRect): Pair<Float, Float> =
        Pair(box.centerX, box.bottom)

    /**
     * Projects a collection of [TrackedObject]s into a list of [WorldObject]s.
     *
     * @param tracks Active tracked objects from the perception/tracker layer.
     * @param pose Current estimated rover pose.
     * @param nowMs Current monotonic/wall timestamp in milliseconds.
     * @param maxCoastAgeMs Maximum staleness in milliseconds for coasting tracks before removal.
     * @return Immutable list of valid semantic [WorldObject]s.
     */
    fun projectTracks(
        tracks: List<TrackedObject>,
        pose: LocalPose,
        nowMs: Long = System.currentTimeMillis(),
        maxCoastAgeMs: Long = MAX_COAST_AGE_MS,
    ): List<WorldObject> {
        if (tracks.isEmpty()) return emptyList()

        val results = ArrayList<WorldObject>(tracks.size)

        for (track in tracks) {
            // Filter unconfirmed/tentative tracks
            if (!track.isConfirmed) continue

            // Filter stale coasting tracks
            val stalenessMs = (nowMs - track.lastSeenMs).coerceAtLeast(0L)
            if (track.isCoasting && stalenessMs > maxCoastAgeMs) continue

            val worldObj = projectSingleTrack(track, pose)
            results.add(worldObj)
        }

        return results
    }

    /**
     * Projects a single [TrackedObject] given the current [LocalPose].
     */
    fun projectSingleTrack(
        track: TrackedObject,
        pose: LocalPose,
    ): WorldObject {
        // 1. Image Ground Contact (bottom-center)
        val groundContact = calculateGroundContact(track.boundingBox)

        // 2. Camera Optical Bearing -> Rover Body Frame
        // Optical: 0° boresight, -left, +right
        // Rover: +X forward, +Y left (CCW positive: +left, -right)
        val roverBearingDeg = Units.normalizeDeg(-track.bearingDeg)
        val roverBearingRad = roverBearingDeg * Units.DEG_TO_RAD

        // 3. Distance & Metric Position Estimation
        val distanceM = track.estimatedDistanceM
        val hasValidDistance = distanceM > 0.0f && distanceM.isFinite()

        val roverXM: Float?
        val roverYM: Float?
        val roverDistanceM: Float?
        val distanceCertainty: DistanceCertainty

        if (hasValidDistance) {
            roverDistanceM = distanceM
            roverXM = distanceM * cos(roverBearingRad.toDouble()).toFloat()
            roverYM = distanceM * sin(roverBearingRad.toDouble()).toFloat()
            distanceCertainty = DistanceCertainty.ESTIMATED_PRIOR
        } else {
            roverDistanceM = null
            roverXM = null
            roverYM = null
            distanceCertainty = DistanceCertainty.UNKNOWN
        }

        // 4. World Frame SE(2) Transformation
        val mapXM: Float?
        val mapYM: Float?

        val isPoseScaleValid = pose.scaleState == MonocularScaleState.SCALE_ESTIMATED
        val isPoseTrackingValid = pose.trackingState == TrackingQuality.TRACKING ||
                (pose.trackingState == TrackingQuality.DEGRADED && pose.confidence >= 0.40f)

        if (hasValidDistance && roverXM != null && roverYM != null && isPoseScaleValid && isPoseTrackingValid) {
            val yawRad = pose.yawDeg * Units.DEG_TO_RAD
            val cosYaw = cos(yawRad.toDouble()).toFloat()
            val sinYaw = sin(yawRad.toDouble()).toFloat()

            // SE(2) Rigid Body Transformation:
            // X_world = X_pose + (roverX * cos(yaw) - roverY * sin(yaw))
            // Y_world = Y_pose + (roverX * sin(yaw) + roverY * cos(yaw))
            mapXM = pose.xM + (roverXM * cosYaw - roverYM * sinYaw)
            mapYM = pose.yM + (roverXM * sinYaw + roverYM * cosYaw)
        } else {
            mapXM = null
            mapYM = null
        }

        val trackIdInt = track.id.toIntOrNull() ?: track.id.hashCode()

        return WorldObject(
            trackId = trackIdInt,
            label = track.label,
            objectType = track.type,
            confidence = track.confidence,
            timestampMs = track.lastSeenMs,
            roverDistanceM = roverDistanceM,
            roverBearingDeg = roverBearingDeg,
            roverXM = roverXM,
            roverYM = roverYM,
            mapXM = mapXM,
            mapYM = mapYM,
            widthM = if (track.metricWidthM > 0f) track.metricWidthM else null,
            heightM = if (track.metricHeightM > 0f) track.metricHeightM else null,
            relativeMotion = track.relativeMotion,
            isConfirmed = track.isConfirmed,
            isCoasting = track.isCoasting,
            threatScore = track.threatScore,
            distanceCertainty = distanceCertainty,
        )
    }
}
