package com.minesafety.roboeye.vision

import com.minesafety.roboeye.core.ImuReading
import com.minesafety.roboeye.core.model.CameraFrame
import com.minesafety.roboeye.core.model.FreeSpaceObservation
import com.minesafety.roboeye.core.model.ObstacleObservation
import com.minesafety.roboeye.fusion.CorridorSector
import com.minesafety.roboeye.fusion.FreeSpaceClearance
import com.minesafety.roboeye.fusion.GeometricFreeSpaceEstimator
import com.minesafety.roboeye.fusion.VisualFreeSpaceEvidence
import kotlin.math.abs

/**
 * Production-ready lightweight Camera + IMU Visual-Inertial Geometric Vision Pipeline.
 *
 * Implements [VisionPipeline] for real-time low-overhead perception on budget hardware
 * (e.g. Redmi 6A @ 10 FPS, 640x360 NHD):
 * 1. Shi-Tomasi or FAST feature detection on direct Y-plane
 * 2. Lucas-Kanade sparse pyramidal optical flow tracking
 * 3. Flow validation (bounds, noise floor, statistical IQR outlier filtering)
 * 4. RANSAC 2D displacement consensus filter for outlier rejection
 * 5. Phone IMU gyroscope rotational flow compensation
 * 6. Camera / IMU motion consistency evaluation
 * 7. Deterministic visual-inertial motion state classification
 * 8. IMU-aware Focus of Expansion (FOE) and Time-To-Collision (TTC) estimation
 * 9. Geometric Free-Space & 3-Corridor traversability evaluation
 * 10. Local 2D Visual Occupancy Grid with temporal persistence
 * 11. Scene geometry reliability and trust assessment
 */
class GeometricVisionPipeline(
    val detector: FeatureDetector = ShiTomasiFeatureDetector(),
    val tracker: OpticalFlowEstimator = LucasKanadeOpticalFlow(),
    val validator: FlowValidator = FlowValidator(),
    val ransacFilter: RansacFlowFilter = RansacFlowFilter(),
    val rotationCompensator: ImuRotationCompensator = ImuRotationCompensator(),
    val consistencyChecker: VisualImuConsistencyChecker = VisualImuConsistencyChecker(),
    val motionEstimator: VisualInertialMotionEstimator = VisualInertialMotionEstimator(),
    val foeEstimator: RobustFoeEstimator = RobustFoeEstimator(),
    val ttcEstimator: RegionalTtcEstimator = RegionalTtcEstimator(),
    val freeSpaceEstimator: GeometricFreeSpaceEstimator = GeometricFreeSpaceEstimator(),
    val occupancyGrid: LocalVisualOccupancyGrid = LocalVisualOccupancyGrid(),
    val geometryAssessor: SceneGeometryAssessor = SceneGeometryAssessor(),
    val localizer: com.minesafety.roboeye.localization.VisualInertialLocalizer? = null,
    val spatialMapper: com.minesafety.roboeye.mapping.LocalSpatialMapper? = null,
    val maxFeatures: Int = 150,
    val minFeatureReplenishThreshold: Int = 40,
    var currentImuProvider: (() -> ImuReading?)? = null,
) : VisionPipeline {

    override val isReady: Boolean = true

    private var previousFrame: CameraFrame? = null
    private var previousPoints: List<FeaturePoint> = emptyList()

    /** Callback for detailed geometric and visual-inertial diagnostics. */
    var onGeometricResult: ((GeometricVisionResult) -> Unit)? = null

    @Synchronized
    override fun processFrame(frame: CameraFrame): VisionOutput {
        val imu = currentImuProvider?.invoke()
        return processFrame(frame, imu)
    }

    @Synchronized
    fun processFrame(frame: CameraFrame, imuReading: ImuReading?): VisionOutput {
        val startTimeMs = System.currentTimeMillis()
        val yBuf = frame.yBuffer

        // If no direct Y-buffer is available, return empty output
        if (yBuf == null) {
            return VisionOutput(
                frameSequence = frame.sequenceNumber,
                timestampNs = frame.timestampNs,
                processingTimeMs = System.currentTimeMillis() - startTimeMs,
            )
        }

        val prev = previousFrame
        if (prev == null || previousPoints.isEmpty()) {
            // First frame or after reset: detect initial features
            val initialPoints = detector.detect(frame, maxFeatures)
            previousFrame = frame
            previousPoints = initialPoints

            val initGeoResult = GeometricVisionResult(
                detectedFeaturesCount = initialPoints.size,
                trackedFeaturesCount = 0,
                validFlowCount = 0,
                foeX = frame.width / 2.0f,
                foeY = frame.height / 2.0f,
                isDivergent = false,
                foeConfidence = 0.0f,
                ttcResult = RegionalTtcResult(
                    left = SectorTtc("LEFT", null, null, SectorRisk.SAFE, 0),
                    center = SectorTtc("CENTER", null, null, SectorRisk.SAFE, 0),
                    right = SectorTtc("RIGHT", null, null, SectorRisk.SAFE, 0),
                ),
                processingTimeMs = System.currentTimeMillis() - startTimeMs,
            )
            onGeometricResult?.invoke(initGeoResult)
            android.util.Log.i("RoboEyeGeo", "GeometricVisionPipeline: Initialized with ${initialPoints.size} features on frame #${frame.sequenceNumber}")

            return VisionOutput(
                frameSequence = frame.sequenceNumber,
                timestampNs = frame.timestampNs,
                processingTimeMs = System.currentTimeMillis() - startTimeMs,
            )
        }

        // Replenish features if point count has dropped
        var currentPointsToTrack = previousPoints
        if (currentPointsToTrack.size < minFeatureReplenishThreshold) {
            val newPoints = detector.detect(prev, maxFeatures - currentPointsToTrack.size)
            currentPointsToTrack = currentPointsToTrack + newPoints
        }

        // Step 1: Track features via Lucas-Kanade optical flow
        val rawVectors = tracker.track(prev, frame, currentPointsToTrack)

        // Step 2: Validate flow vectors and filter noise/bounds/IQR outliers
        val validation = validator.validate(rawVectors, frame.width, frame.height)
        val validVectors = validation.validVectors

        // Step 3: RANSAC geometric flow consensus filtering
        val ransacResult = ransacFilter.filter(validVectors)
        val inlierVectors = ransacResult.inliers

        // Step 4: Elapsed time calculation
        val dtSec = if (prev.timestampNs > 0 && frame.timestampNs > prev.timestampNs) {
            ((frame.timestampNs - prev.timestampNs) / 1_000_000_000.0f).coerceIn(0.02f, 0.5f)
        } else {
            0.1f // default to 10 FPS nominal
        }

        // Step 5: IMU telemetry freshness & rotational motion compensation
        val isFreshImu = imuReading != null && imuReading.isAvailable && (
            frame.timestampNs == 0L || imuReading.timestampNs == 0L ||
            abs(frame.timestampNs - imuReading.timestampNs) <= 250_000_000L
        )

        val compensatedFlow = rotationCompensator.compensate(
            vectors = inlierVectors,
            imu = if (isFreshImu) imuReading else null,
            dtSec = dtSec,
            imageWidth = frame.width,
            imageHeight = frame.height,
        )
        val motionVectors = compensatedFlow.compensatedVectors

        // Step 6: Camera / IMU Consistency check
        val consistency = consistencyChecker.check(
            inlierVectors = inlierVectors,
            meanDx = ransacResult.consensusDx,
            meanDy = ransacResult.consensusDy,
            imu = imuReading,
            isFreshImu = isFreshImu,
        )

        // Step 7: Estimate FOE on rotation-compensated inlier flow
        val rawFoe = foeEstimator.estimate(motionVectors, frame.width, frame.height)
        val foeDampingFactor = compensatedFlow.confidenceMultiplier *
            (if (consistency.state == VisualImuConsistency.CONFLICTING) 0.5f else 1.0f)
        val foe = rawFoe.copy(confidence = (rawFoe.confidence * foeDampingFactor).coerceIn(0.0f, 1.0f))

        // Step 8: Estimate Regional TTC on rotation-compensated vectors
        val rawTtc = ttcEstimator.estimate(
            vectors = motionVectors,
            foeX = foe.x,
            foeY = foe.y,
            isDivergent = foe.isDivergent,
            deltaSec = dtSec,
            imageWidth = frame.width,
            imageHeight = frame.height,
        )

        // Suppress critical TTC alarm if rotation is dominant or sensors conflict
        val effectiveOverallRisk = if (compensatedFlow.isRotationDominant && rawTtc.overallRisk == SectorRisk.CRITICAL) {
            SectorRisk.CAUTION
        } else {
            rawTtc.overallRisk
        }
        val ttc = rawTtc.copy(overallRisk = effectiveOverallRisk)

        // Step 9: Estimate Visual-Inertial Motion State
        val motionEstimate = motionEstimator.estimate(
            ransacResult = ransacResult,
            foe = foe,
            compensatedFlow = compensatedFlow,
            consistencyResult = consistency,
            imu = imuReading,
            isFreshImu = isFreshImu,
            imageWidth = frame.width,
            imageHeight = frame.height,
        )

        // Step 10: Formulate VisualFreeSpaceEvidence and evaluate traversable corridors
        fun computeSectorClearanceM(risk: SectorRisk, minTtc: Float?): Float = when (risk) {
            SectorRisk.SAFE -> 3.8f
            SectorRisk.CAUTION -> ((minTtc ?: 4.0f) * 0.5f).coerceIn(1.8f, 3.8f)
            SectorRisk.WARNING -> ((minTtc ?: 2.0f) * 0.5f).coerceIn(0.8f, 1.8f)
            SectorRisk.CRITICAL -> ((minTtc ?: 1.0f) * 0.5f).coerceIn(0.2f, 0.8f)
        }

        val evLeft = VisualFreeSpaceEvidence(
            sector = CorridorSector.LEFT,
            confidence = (foe.confidence * motionEstimate.confidence).coerceIn(0f, 1f),
            isFresh = true,
            clearanceDistanceM = computeSectorClearanceM(ttc.left.risk, ttc.left.minTtcSec),
            risk = ttc.left.risk,
            featureCount = ttc.left.featureCount,
            isDivergent = foe.isDivergent,
            minTtcSec = ttc.left.minTtcSec,
        )
        val evCenter = VisualFreeSpaceEvidence(
            sector = CorridorSector.CENTER,
            confidence = (foe.confidence * motionEstimate.confidence).coerceIn(0f, 1f),
            isFresh = true,
            clearanceDistanceM = computeSectorClearanceM(ttc.center.risk, ttc.center.minTtcSec),
            risk = ttc.center.risk,
            featureCount = ttc.center.featureCount,
            isDivergent = foe.isDivergent,
            minTtcSec = ttc.center.minTtcSec,
        )
        val evRight = VisualFreeSpaceEvidence(
            sector = CorridorSector.RIGHT,
            confidence = (foe.confidence * motionEstimate.confidence).coerceIn(0f, 1f),
            isFresh = true,
            clearanceDistanceM = computeSectorClearanceM(ttc.right.risk, ttc.right.minTtcSec),
            risk = ttc.right.risk,
            featureCount = ttc.right.featureCount,
            isDivergent = foe.isDivergent,
            minTtcSec = ttc.right.minTtcSec,
        )

        val fusedFreeSpace = freeSpaceEstimator.estimateFromVision(
            left = evLeft,
            center = evCenter,
            right = evRight,
            motionState = motionEstimate.motionState,
            cameraPitchDeg = imuReading?.pitchDeg ?: 0.0f,
        )

        // Step 11: Update Local Visual Occupancy Grid
        val occupancySummary = occupancyGrid.update(
            evidenceList = listOf(evLeft, evCenter, evRight),
            motionState = motionEstimate.motionState,
            yawRateDps = motionEstimate.rotationalVelocityDps,
        )

        // Step 12: Assess Scene Geometry Reliability
        val geometryTrust = geometryAssessor.assess(
            consistency = consistency.state,
            motionState = motionEstimate.motionState,
            inlierCount = inlierVectors.size,
            yawRateDps = motionEstimate.rotationalVelocityDps,
            pitchDeg = imuReading?.pitchDeg ?: 0.0f,
            rollDeg = imuReading?.rollDeg ?: 0.0f,
        )

        // Step 13: Maintain surviving points for next frame tracking
        val survivingPoints = if (inlierVectors.isNotEmpty()) {
            inlierVectors.map { v ->
                FeaturePoint(v.currX, v.currY, response = v.confidence)
            }
        } else {
            // When inlier count drops to 0 (e.g. static scene or abrupt movement),
            // immediately detect fresh features on current frame so the next frame is never empty.
            detector.detect(frame, maxFeatures)
        }
        previousPoints = survivingPoints
        previousFrame = frame

        // Step 14: Visual-Inertial Localization & Local Spatial Mapping
        val localPose = localizer?.update(
            ransacResult = ransacResult,
            foe = foe,
            compensatedFlow = compensatedFlow,
            motionState = motionEstimate.motionState,
            geometryTrust = geometryTrust.trustLevel,
            imageWidth = frame.width,
            imageHeight = frame.height,
            dtSec = dtSec,
            timestampNs = frame.timestampNs,
            imuReading = if (isFreshImu) imuReading else null,
            isFreshImu = isFreshImu,
            frame = frame,
            keyframes = spatialMapper?.spatialMap?.storedKeyframes ?: emptyList(),
        )

        val mapState = if (localPose != null && spatialMapper != null) {
            spatialMapper.updateFromVision(
                occupancyCells = occupancyGrid.cells,
                roverPose = localPose,
                landmarks = localizer.landmarkManager.activeLandmarks,
                timestampNs = frame.timestampNs,
            )
        } else null

        val elapsedMs = System.currentTimeMillis() - startTimeMs

        // Step 15: Notify diagnostics callback with enriched Phase 4 & 5 metrics
        val geoResult = GeometricVisionResult(
            detectedFeaturesCount = currentPointsToTrack.size,
            trackedFeaturesCount = rawVectors.size,
            validFlowCount = validVectors.size,
            foeX = foe.x,
            foeY = foe.y,
            isDivergent = foe.isDivergent,
            foeConfidence = foe.confidence,
            ttcResult = ttc,
            processingTimeMs = elapsedMs,
            motionState = motionEstimate.motionState,
            consistency = motionEstimate.consistency,
            motionConfidence = motionEstimate.confidence,
            inlierCount = motionEstimate.inlierCount,
            rotationalDps = motionEstimate.rotationalVelocityDps,
            isImuAvailable = motionEstimate.isImuAvailable,
            ransacIterations = ransacResult.iterationsRun,
            freeSpace = fusedFreeSpace,
            occupancySummary = occupancySummary,
            geometryTrust = geometryTrust.trustLevel,
            recommendedCorridor = fusedFreeSpace.recommendedCorridor,
            localPose = localPose,
            mapState = mapState,
        )
        onGeometricResult?.invoke(geoResult)
        if (frame.sequenceNumber % 10L == 0L) {
            android.util.Log.i("RoboEyeGeo", "Pipeline pass #${frame.sequenceNumber}: detected=${currentPointsToTrack.size}, tracked=${rawVectors.size}, inliers=${inlierVectors.size}, motion=${motionEstimate.motionState}, trust=${geometryTrust.trustLevel}, sync=${consistency.state}")
        }

        // Step 15: Map TTC risks into ObstacleObservation and FreeSpaceObservation
        val obstacles = ArrayList<ObstacleObservation>()
        if (ttc.overallRisk == SectorRisk.CRITICAL || ttc.overallRisk == SectorRisk.WARNING) {
            val bearing = when (ttc.criticalSector) {
                "LEFT" -> -25.0f
                "RIGHT" -> 25.0f
                else -> 0.0f
            }
            val approxDistance = (ttc.minTtcSec ?: 1.0f) * 0.5f

            obstacles.add(
                ObstacleObservation(
                    id = "FLOW_OBS_${frame.sequenceNumber}",
                    distanceM = approxDistance,
                    bearingDeg = bearing,
                    timeToCollisionSec = ttc.minTtcSec,
                    source = "OPTICAL_FLOW",
                    confidence = foe.confidence * motionEstimate.confidence,
                    timestampMs = System.currentTimeMillis(),
                )
            )
        }

        val freeSpace = FreeSpaceObservation(
            isSupported = true,
            clearDistanceAheadM = fusedFreeSpace.center.clearDistanceM,
            traversableWidthLeftM = if (fusedFreeSpace.left.clearance == FreeSpaceClearance.CLEAR_WITH_CONFIDENCE) 1.2f else 0.0f,
            traversableWidthRightM = if (fusedFreeSpace.right.clearance == FreeSpaceClearance.CLEAR_WITH_CONFIDENCE) 1.2f else 0.0f,
            isPassable = fusedFreeSpace.isClearAhead,
            confidence = fusedFreeSpace.overallConfidence,
            timestampMs = System.currentTimeMillis(),
        )

        return VisionOutput(
            frameSequence = frame.sequenceNumber,
            timestampNs = frame.timestampNs,
            obstacles = obstacles,
            freeSpace = freeSpace,
            processingTimeMs = elapsedMs,
        )
    }

    /**
     * Resets temporal feature history and frame buffer (e.g. on camera restart or mode switch).
     */
    @Synchronized
    fun reset() {
        previousFrame = null
        previousPoints = emptyList()
        motionEstimator.reset()
        occupancyGrid.reset()
    }
}
