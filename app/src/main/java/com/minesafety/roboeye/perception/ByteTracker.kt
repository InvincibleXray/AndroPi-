package com.minesafety.roboeye.perception

import com.minesafety.roboeye.core.ImuReading
import com.minesafety.roboeye.core.NormalizedRect
import com.minesafety.roboeye.core.PerceptionObjectType
import com.minesafety.roboeye.core.RawDetectedObject
import com.minesafety.roboeye.core.RelativeMotion
import com.minesafety.roboeye.core.TrackedObject
import com.minesafety.roboeye.core.Units
import java.util.Locale
import java.util.concurrent.atomic.AtomicInteger
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.min

/**
 * High-performance Multi-Object Tracker based on the ByteTrack association algorithm
 * augmented with IMU Camera Motion Compensation (IMU-CMC) for rough terrain.
 *
 * Key Capabilities:
 *  1. Two-Stage Association: Matches high-confidence detections (>= 0.50) first,
 *     then recovers partially occluded or low-visibility detections (0.15 .. 0.50)
 *     using remaining tracks, preventing track drops in mine dust and fog.
 *  2. IMU-CMC: Compensates for rover bounce and steering jitter using phone gyroscope
 *     rotational rates (yaw rate and pitch changes) before Kalman prediction association.
 *  3. Kinematic state estimation: Tracks position, velocity, and relative motion.
 *  4. Metric Physical Footprint & Mass: Populates physical width, height, and mass via PhysicalEstimator.
 *  5. Configurable parameters with sensible mine-safety defaults.
 */
class ByteTracker(
    val highConfidenceThreshold: Float = 0.50f,
    val lowConfidenceThreshold: Float = 0.15f,
    val iouThreshold: Float = 0.30f,
    val trackBufferFrames: Int = 15,
    val maxCoastingAgeMs: Long = 1000L,
    val highConfidenceImmediateThreshold: Float = 0.75f,
    val minHitsToConfirm: Int = 2,
    var enableImuCmc: Boolean = true,
) : Tracker {

    override val trackerType: TrackerType = TrackerType.BYTE_TRACK

    enum class TrackLifecycleState {
        TENTATIVE,
        CONFIRMED,
        LOST,
        REMOVED,
    }

    private class InternalTrack(
        val id: String,
        var type: PerceptionObjectType,
        var label: String,
        var box: NormalizedRect,
        var vx: Float = 0.0f,
        var vy: Float = 0.0f,
        var bearingDeg: Float = 0.0f,
        var estimatedDistanceM: Float = 1.0f,
        var confidence: Float = 0.5f,
        var state: TrackLifecycleState = TrackLifecycleState.TENTATIVE,
        var hitCount: Int = 1,
        var lostCount: Int = 0,
        val firstSeenMs: Long = System.currentTimeMillis(),
        var lastSeenMs: Long = System.currentTimeMillis(),
        var relativeMotion: RelativeMotion = RelativeMotion.STATIONARY,
        var metricWidthM: Float = 0.0f,
        var metricHeightM: Float = 0.0f,
        var footprintLeftM: Float = 0.0f,
        var footprintRightM: Float = 0.0f,
        var estimatedMassKg: Float = 0.0f,
    )

    private val idCounter = AtomicInteger(1)
    private val tracks = mutableListOf<InternalTrack>()
    private var lastUpdateMs: Long = 0L
    private var lastImuPitchDeg: Float = 0.0f

    @Synchronized
    override fun update(
        detections: List<RawDetectedObject>,
        cameraHfovDeg: Float,
        cameraVfovDeg: Float,
        cameraHeightM: Float,
        imuPitchDeg: Float,
        imuReading: ImuReading,
        nowMs: Long,
    ): List<TrackedObject> {
        val dtSec = if (lastUpdateMs > 0L) {
            ((nowMs - lastUpdateMs).coerceIn(10L, 500L)) / 1000.0f
        } else 0.033f
        lastUpdateMs = nowMs

        // 1. Camera Motion Compensation (IMU-CMC)
        var deltaXNorm = 0.0f
        var deltaYNorm = 0.0f
        if (enableImuCmc && imuReading.isAvailable && cameraHfovDeg > 1.0f && cameraVfovDeg > 1.0f) {
            val deltaPitch = imuPitchDeg - lastImuPitchDeg
            val deltaYaw = imuReading.yawRateDps * dtSec

            // Small angle approximation for pixel displacement in normalized coordinates
            deltaXNorm = (deltaYaw / cameraHfovDeg).coerceIn(-0.3f, 0.3f)
            deltaYNorm = (deltaPitch / cameraVfovDeg).coerceIn(-0.3f, 0.3f)
        }
        lastImuPitchDeg = imuPitchDeg

        // 2. Predict existing tracks forward and apply IMU motion compensation
        for (track in tracks) {
            if (track.state != TrackLifecycleState.REMOVED) {
                // Kinematic velocity prediction
                val predCenterX = (track.box.centerX + track.vx * dtSec - deltaXNorm).coerceIn(0.0f, 1.0f)
                val predCenterY = (track.box.centerY + track.vy * dtSec - deltaYNorm).coerceIn(0.0f, 1.0f)
                val halfW = track.box.width * 0.5f
                val halfH = track.box.height * 0.5f

                track.box = NormalizedRect(
                    left = (predCenterX - halfW).coerceIn(0.0f, 1.0f),
                    top = (predCenterY - halfH).coerceIn(0.0f, 1.0f),
                    right = (predCenterX + halfW).coerceIn(0.0f, 1.0f),
                    bottom = (predCenterY + halfH).coerceIn(0.0f, 1.0f),
                )
            }
        }

        // 3. Partition detections into High-Confidence and Low-Confidence Pools
        val dHigh = detections.filter { it.confidence >= highConfidenceThreshold }
        val dLow = detections.filter { it.confidence in lowConfidenceThreshold..<highConfidenceThreshold }

        val activeConfirmed = tracks.filter { it.state == TrackLifecycleState.CONFIRMED || it.state == TrackLifecycleState.LOST }.toMutableList()

        // -------------------------------------------------------------
        // FIRST ASSOCIATION: High-Confidence Detections with Active Tracks
        // -------------------------------------------------------------
        val matchedHighPairs = matchBoxes(activeConfirmed.map { it.box }, dHigh.map { it.boundingBox }, iouThreshold)
        val matchedHighTracks = mutableSetOf<InternalTrack>()
        val matchedHighDetections = mutableSetOf<RawDetectedObject>()

        for ((trackIdx, detIdx) in matchedHighPairs) {
            val track = activeConfirmed[trackIdx]
            val det = dHigh[detIdx]
            updateTrackWithDetection(track, det, cameraHfovDeg, cameraVfovDeg, cameraHeightM, imuPitchDeg, dtSec, nowMs)
            matchedHighTracks.add(track)
            matchedHighDetections.add(det)
        }

        val unmatchedConfirmedAfterStage1 = activeConfirmed.filter { it !in matchedHighTracks }
        val unmatchedHighDetections = dHigh.filter { it !in matchedHighDetections }

        // -------------------------------------------------------------
        // SECOND ASSOCIATION: Low-Confidence Detections with Unmatched Confirmed Tracks (ByteTrack Core)
        // -------------------------------------------------------------
        val matchedLowPairs = matchBoxes(unmatchedConfirmedAfterStage1.map { it.box }, dLow.map { it.boundingBox }, iouThreshold)
        val matchedLowTracks = mutableSetOf<InternalTrack>()

        for ((trackIdx, detIdx) in matchedLowPairs) {
            val track = unmatchedConfirmedAfterStage1[trackIdx]
            val det = dLow[detIdx]
            updateTrackWithDetection(track, det, cameraHfovDeg, cameraVfovDeg, cameraHeightM, imuPitchDeg, dtSec, nowMs)
            matchedLowTracks.add(track)
        }

        val remainingUnmatchedConfirmed = unmatchedConfirmedAfterStage1.filter { it !in matchedLowTracks }

        // -------------------------------------------------------------
        // THIRD ASSOCIATION: Unmatched High Detections with Tentative Tracks
        // -------------------------------------------------------------
        val tentativeTracks = tracks.filter { it.state == TrackLifecycleState.TENTATIVE }.toMutableList()
        val matchedTentativePairs = matchBoxes(tentativeTracks.map { it.box }, unmatchedHighDetections.map { it.boundingBox }, iouThreshold)
        val matchedTentativeTracks = mutableSetOf<InternalTrack>()
        val matchedTentativeDets = mutableSetOf<RawDetectedObject>()

        for ((trackIdx, detIdx) in matchedTentativePairs) {
            val track = tentativeTracks[trackIdx]
            val det = unmatchedHighDetections[detIdx]
            updateTrackWithDetection(track, det, cameraHfovDeg, cameraVfovDeg, cameraHeightM, imuPitchDeg, dtSec, nowMs)
            matchedTentativeTracks.add(track)
            matchedTentativeDets.add(det)
        }

        // Remaining unmatched tentative tracks age out quickly
        for (tentative in tentativeTracks) {
            if (tentative !in matchedTentativeTracks) {
                tentative.state = TrackLifecycleState.REMOVED
            }
        }

        // Remaining unmatched high-confidence detections initialize new tentative tracks
        val brandNewDetections = unmatchedHighDetections.filter { it !in matchedTentativeDets }
        for (det in brandNewDetections) {
            val newId = nextTrackId()
            val rawBearing = BearingEstimator.calculateBearingDeg(det.boundingBox, cameraHfovDeg)
            val rawDistance = DistanceEstimator.estimateDistanceM(
                det.boundingBox,
                det.type,
                cameraVfovDeg,
                cameraHeightM,
                imuPitchDeg
            )
            val metrics = PhysicalEstimator.estimateMetrics(
                det.boundingBox,
                det.type,
                rawDistance,
                cameraHfovDeg,
                cameraVfovDeg
            )
            val isImmediate = det.confidence >= highConfidenceImmediateThreshold

            tracks.add(
                InternalTrack(
                    id = newId,
                    type = det.type,
                    label = det.label,
                    box = det.boundingBox,
                    bearingDeg = rawBearing,
                    estimatedDistanceM = rawDistance,
                    confidence = Units.round2(det.confidence),
                    state = if (isImmediate) TrackLifecycleState.CONFIRMED else TrackLifecycleState.TENTATIVE,
                    hitCount = 1,
                    firstSeenMs = nowMs,
                    lastSeenMs = nowMs,
                    relativeMotion = RelativeMotion.STATIONARY,
                    metricWidthM = metrics.widthM,
                    metricHeightM = metrics.heightM,
                    footprintLeftM = metrics.footprintLeftX,
                    footprintRightM = metrics.footprintRightX,
                    estimatedMassKg = metrics.estimatedMassKg,
                )
            )
        }

        // -------------------------------------------------------------
        // 4. Update Lost / Coasting State & Prune Dead Tracks
        // -------------------------------------------------------------
        for (unmatched in remainingUnmatchedConfirmed) {
            unmatched.lostCount++
            unmatched.state = TrackLifecycleState.LOST
            val timeSinceSeen = nowMs - unmatched.lastSeenMs

            if (unmatched.lostCount > trackBufferFrames || timeSinceSeen > maxCoastingAgeMs) {
                unmatched.state = TrackLifecycleState.REMOVED
            } else {
                // Decay confidence while coasting
                unmatched.confidence = Units.round2((unmatched.confidence * 0.92f).coerceAtLeast(0.15f))
                if (timeSinceSeen > 400L) {
                    unmatched.relativeMotion = RelativeMotion.STATIONARY
                }
            }
        }

        // Clean up removed tracks
        tracks.removeAll { it.state == TrackLifecycleState.REMOVED }

        // -------------------------------------------------------------
        // 5. Build Confirmed / Coasting Output Targets
        // -------------------------------------------------------------
        val activeVisible = tracks.filter { it.state == TrackLifecycleState.CONFIRMED || it.state == TrackLifecycleState.LOST }

        val outputTargets = activeVisible.map { track ->
            TrackedObject(
                id = track.id,
                type = track.type,
                label = track.label,
                boundingBox = track.box,
                bearingDeg = track.bearingDeg,
                estimatedDistanceM = track.estimatedDistanceM,
                confidence = track.confidence,
                relativeMotion = track.relativeMotion,
                firstSeenMs = track.firstSeenMs,
                lastSeenMs = track.lastSeenMs,
                hitCount = track.hitCount,
                isConfirmed = (track.state == TrackLifecycleState.CONFIRMED),
                isPrimary = false,
                threatScore = 0.0f,
                isCoasting = (track.state == TrackLifecycleState.LOST),
                metricWidthM = track.metricWidthM,
                metricHeightM = track.metricHeightM,
                groundFootprintLeftM = track.footprintLeftM,
                groundFootprintRightM = track.footprintRightM,
                estimatedMassKg = track.estimatedMassKg,
                isOnRoad = true,
                blockageContributionPct = 0.0f,
            )
        }

        return prioritizeTargets(outputTargets)
    }

    private fun updateTrackWithDetection(
        track: InternalTrack,
        det: RawDetectedObject,
        cameraHfovDeg: Float,
        cameraVfovDeg: Float,
        cameraHeightM: Float,
        imuPitchDeg: Float,
        dtSec: Float,
        nowMs: Long,
    ) {
        val rawBearing = BearingEstimator.calculateBearingDeg(det.boundingBox, cameraHfovDeg)
        val rawDistance = DistanceEstimator.estimateDistanceM(
            det.boundingBox,
            det.type,
            cameraVfovDeg,
            cameraHeightM,
            imuPitchDeg
        )

        // Adaptive Bearing Smoothing
        val bearingDelta = abs(rawBearing - track.bearingDeg)
        val bearingAlpha = if (bearingDelta < 2.5f) 0.30f else 0.70f
        track.bearingDeg = Units.round1(bearingAlpha * rawBearing + (1.0f - bearingAlpha) * track.bearingDeg)

        // Distance Outlier Rejection & Smoothing
        val maxStepM = max(2.0f, dtSec * 5.0f)
        val diffM = (rawDistance - track.estimatedDistanceM).coerceIn(-maxStepM, maxStepM)
        val filteredRaw = track.estimatedDistanceM + diffM
        val distAlpha = 0.40f
        val newDistance = Units.round1(distAlpha * filteredRaw + (1.0f - distAlpha) * track.estimatedDistanceM)

        // Velocity Estimation
        val deltaDist = newDistance - track.estimatedDistanceM
        val rangeRateMps = deltaDist / dtSec
        track.relativeMotion = when {
            rangeRateMps < -0.30f -> RelativeMotion.APPROACHING
            rangeRateMps > 0.30f -> RelativeMotion.RECEDING
            else -> RelativeMotion.STATIONARY
        }
        track.estimatedDistanceM = newDistance

        // Bounding Box & Kinematic Velocity Update
        val oldCenterX = track.box.centerX
        val oldCenterY = track.box.centerY
        val newCenterX = det.boundingBox.centerX
        val newCenterY = det.boundingBox.centerY

        val measuredVx = (newCenterX - oldCenterX) / dtSec
        val measuredVy = (newCenterY - oldCenterY) / dtSec
        track.vx = 0.3f * measuredVx + 0.7f * track.vx
        track.vy = 0.3f * measuredVy + 0.7f * track.vy

        // Box Exponential Moving Average (EMA)
        val alphaBox = 0.50f
        track.box = NormalizedRect(
            left = alphaBox * det.boundingBox.left + (1.0f - alphaBox) * track.box.left,
            top = alphaBox * det.boundingBox.top + (1.0f - alphaBox) * track.box.top,
            right = alphaBox * det.boundingBox.right + (1.0f - alphaBox) * track.box.right,
            bottom = alphaBox * det.boundingBox.bottom + (1.0f - alphaBox) * track.box.bottom,
        )

        // Confidence EMA
        val confAlpha = 0.40f
        track.confidence = Units.round2(confAlpha * det.confidence + (1.0f - confAlpha) * track.confidence)

        // Metric Physical Footprint
        val metrics = PhysicalEstimator.estimateMetrics(
            track.box,
            det.type,
            newDistance,
            cameraHfovDeg,
            cameraVfovDeg
        )
        track.metricWidthM = metrics.widthM
        track.metricHeightM = metrics.heightM
        track.footprintLeftM = metrics.footprintLeftX
        track.footprintRightM = metrics.footprintRightX
        track.estimatedMassKg = metrics.estimatedMassKg

        track.type = det.type
        track.label = det.label
        track.hitCount++
        track.lostCount = 0
        track.lastSeenMs = nowMs

        // Confirmation transition
        if (track.hitCount >= minHitsToConfirm || det.confidence >= highConfidenceImmediateThreshold) {
            track.state = TrackLifecycleState.CONFIRMED
        }
    }

    private fun prioritizeTargets(tracks: List<TrackedObject>): List<TrackedObject> {
        if (tracks.isEmpty()) return emptyList()

        val scored = tracks.map { track ->
            val distScore = (1.0f - (track.estimatedDistanceM / 25.0f)).coerceIn(0.0f, 1.0f)
            val bearingScore = (1.0f - (abs(track.bearingDeg) / 35.0f)).coerceIn(0.0f, 1.0f)
            val motionScore = when (track.relativeMotion) {
                RelativeMotion.APPROACHING -> 1.0f
                RelativeMotion.STATIONARY -> 0.5f
                RelativeMotion.UNKNOWN -> 0.3f
                RelativeMotion.RECEDING -> 0.15f
            }
            val confScore = track.confidence.coerceIn(0.0f, 1.0f)
            val stabilityScore = (track.hitCount / 4.0f).coerceIn(0.0f, 1.0f)

            val composite = Units.round2(
                0.40f * distScore +
                0.25f * bearingScore +
                0.15f * motionScore +
                0.10f * confScore +
                0.10f * stabilityScore
            )
            track.copy(threatScore = composite)
        }

        val maxThreat = scored.maxOfOrNull { it.threatScore } ?: 0.0f
        val primaryCandidate = if (maxThreat > 0.10f) scored.maxByOrNull { it.threatScore } else null

        return scored.map { track ->
            track.copy(isPrimary = (track.id == primaryCandidate?.id))
        }
    }

    val activeTrackCount: Int get() = tracks.count { it.state != TrackLifecycleState.REMOVED }

    /**
     * Mints an identifier that is unique for the lifetime of this tracker.
     *
     * The counter used to be taken modulo 100, so the 101st track opened was named `TRK-01`
     * again. That is not a cosmetic wrap: the id is the *identity* of a target, and several
     * consumers compare on it. [prioritizeTargets] marks `isPrimary` by `track.id ==
     * primaryCandidate.id`, [ThreatEngine] returns a `primaryThreatId` the view model matches
     * the same way, and the operator's tap selection in `RadarViewModel` is stored as an id.
     * Two live tracks sharing one would light up as primary together, and deselecting one
     * would deselect the other. A long-lived track — a boulder beside a haul road held for
     * minutes — is precisely the one still present when the counter comes back round.
     *
     * The width is a minimum, not a limit: track 100 formats as `TRK-100`. [Locale.ROOT]
     * because a locale with non-ASCII digits would otherwise mint ids that no longer compare
     * equal to ones minted before the device locale changed.
     */
    private fun nextTrackId(): String =
        String.format(Locale.ROOT, "TRK-%02d", idCounter.getAndIncrement())

    @Synchronized
    override fun reset() {
        tracks.clear()
        idCounter.set(1)
        lastUpdateMs = 0L
    }

    companion object {
        fun calculateIoU(a: NormalizedRect, b: NormalizedRect): Float {
            val interLeft = max(a.left, b.left)
            val interTop = max(a.top, b.top)
            val interRight = min(a.right, b.right)
            val interBottom = min(a.bottom, b.bottom)

            val interWidth = max(0.0f, interRight - interLeft)
            val interHeight = max(0.0f, interBottom - interTop)
            val interArea = interWidth * interHeight

            val areaA = a.width * a.height
            val areaB = b.width * b.height
            val unionArea = areaA + areaB - interArea

            return if (unionArea > 0.0001f) interArea / unionArea else 0.0f
        }

        /**
         * Greedy bipartite IoU matching between track boxes and detection boxes.
         * Returns list of matched (trackIndex, detectionIndex) pairs.
         */
        fun matchBoxes(
            trackBoxes: List<NormalizedRect>,
            detectionBoxes: List<NormalizedRect>,
            threshold: Float,
        ): List<Pair<Int, Int>> {
            if (trackBoxes.isEmpty() || detectionBoxes.isEmpty()) return emptyList()

            val matches = mutableListOf<Pair<Int, Int>>()
            val usedTracks = BooleanArray(trackBoxes.size)
            val usedDetections = BooleanArray(detectionBoxes.size)

            // Compute all pairwise IoU scores
            val candidates = mutableListOf<Triple<Int, Int, Float>>()
            for (t in trackBoxes.indices) {
                for (d in detectionBoxes.indices) {
                    val iou = calculateIoU(trackBoxes[t], detectionBoxes[d])
                    if (iou >= threshold) {
                        candidates.add(Triple(t, d, iou))
                    }
                }
            }

            // Greedy assignment descending by IoU
            candidates.sortByDescending { it.third }
            for ((t, d, _) in candidates) {
                if (!usedTracks[t] && !usedDetections[d]) {
                    usedTracks[t] = true
                    usedDetections[d] = true
                    matches.add(t to d)
                }
            }

            return matches
        }
    }
}
