package com.minesafety.roboeye.perception

import com.minesafety.roboeye.core.RoadGeometryState
import com.minesafety.roboeye.core.RoadStatus
import com.minesafety.roboeye.core.TrackedObject
import com.minesafety.roboeye.core.Units
import kotlin.math.max
import kotlin.math.min

/**
 * Severity level of road blockage.
 */
enum class RoadBlockageSeverity(val label: String) {
    NONE("CLEAR"),
    LOW("MINOR OBSTRUCTION"),
    MODERATE("PARTIALLY BLOCKED"),
    SEVERE("SEVERELY BLOCKED"),
    TOTAL("ROAD IMPASSABLE"),
}

/**
 * Geometric road blockage assessment result.
 *
 * Meaningful only when the [RoadSegmentationResult] it was derived from had
 * `hasValidRoad = true`. The defaults describe "no road, therefore no corridor": zero
 * clearance and not passable, so a caller that ignores the road validity flag fails safe.
 */
data class RoadBlockage(
    val isBlocked: Boolean = false,
    val blockagePct: Float = 0.0f,
    val severity: RoadBlockageSeverity = RoadBlockageSeverity.NONE,
    val blockingObjectIds: List<String> = emptyList(),
    val traversableCorridorLeftM: Float = 0.0f,
    val traversableCorridorRightM: Float = 0.0f,
    val maxClearanceM: Float = 0.0f,
    val isPassable: Boolean = false,
    val estimatedDistanceM: Float = 0.0f,
    val confidence: Float = 0.0f,
)

/**
 * Geometric analyzer that determines whether detected obstacles intersect the drivable road corridor
 * and calculates remaining traversable lane width.
 *
 * Distinct from a naive 2D bounding-box pixel ratio: evaluates true metric ground-plane footprints.
 *
 * **Requires a real road.** Object footprints are measured (they come from the distance and
 * physical-size estimators), but "how much of the road is blocked" is a ratio against a road
 * width, so the result is only as real as its [RoadSegmentationResult] input. No road-perception
 * producer is implemented today, so in practice [analyze] takes its unavailable branch — see
 * [RoadSegmentationEngine].
 */
class RoadBlockageAnalyzer(
    val roverWidthM: Float = 0.55f,     // Prototype rover chassis width
    val safetyMarginM: Float = 0.35f,    // Clearance safety margin on either side
) {
    /**
     * Evaluates tracked objects against the active road boundary geometry.
     */
    fun analyze(
        road: RoadSegmentationResult,
        trackedObjects: List<TrackedObject>,
    ): Pair<RoadBlockage, List<TrackedObject>> {
        if (!road.hasValidRoad) {
            // No road was perceived, so there is no corridor to be clear or blocked. Report
            // zeroes rather than the road width, and leave `isOnRoad` untouched: whether an
            // object stands on the roadway is a road question, and unanswerable here.
            return RoadBlockage(confidence = 0.0f) to trackedObjects
        }

        if (trackedObjects.isEmpty()) {
            val clearRoad = RoadBlockage(
                isBlocked = false,
                blockagePct = 0.0f,
                severity = RoadBlockageSeverity.NONE,
                blockingObjectIds = emptyList(),
                traversableCorridorLeftM = road.drivableWidthAtRoverM * 0.5f,
                traversableCorridorRightM = road.drivableWidthAtRoverM * 0.5f,
                maxClearanceM = road.drivableWidthAtRoverM * 0.5f,
                isPassable = true,
                estimatedDistanceM = 0.0f,
                confidence = road.roadConfidence,
            )
            return clearRoad to trackedObjects
        }

        // Only evaluate confirmed, non-coasting targets in forward path (< 35m)
        val candidateTargets = trackedObjects.filter { it.isConfirmed && it.estimatedDistanceM in 0.5f..35.0f }
        if (candidateTargets.isEmpty()) {
            val clearRoad = RoadBlockage(
                isBlocked = false,
                blockagePct = 0.0f,
                severity = RoadBlockageSeverity.NONE,
                blockingObjectIds = emptyList(),
                traversableCorridorLeftM = road.drivableWidthAtRoverM * 0.5f,
                traversableCorridorRightM = road.drivableWidthAtRoverM * 0.5f,
                maxClearanceM = road.drivableWidthAtRoverM * 0.5f,
                isPassable = true,
                estimatedDistanceM = 0.0f,
                confidence = road.roadConfidence,
            )
            return clearRoad to trackedObjects
        }

        val updatedObjects = mutableListOf<TrackedObject>()
        val blockingIds = mutableListOf<String>()
        var worstBlockagePct = 0.0f
        var minClearance = Float.MAX_VALUE
        var primaryBlockerDist = 0.0f
        var minLeftClearance = Float.MAX_VALUE
        var minRightClearance = Float.MAX_VALUE

        for (target in trackedObjects) {
            val dist = target.estimatedDistanceM
            val roadWidthAtTarget = road.getDrivableWidthAtDistance(dist)
            val halfRoadWidth = roadWidthAtTarget * 0.5f
            val roadLeftX = -halfRoadWidth
            val roadRightX = halfRoadWidth

            // Object ground footprint in lateral meters relative to centerline
            val footLeft = target.groundFootprintLeftM
            val footRight = target.groundFootprintRightM

            // Evaluate intersection with drivable corridor
            val interLeft = max(roadLeftX, footLeft)
            val interRight = min(roadRightX, footRight)
            val isOnRoad = interRight > interLeft

            if (isOnRoad) {
                val blockedWidth = max(0.0f, interRight - interLeft)
                val blockagePct = Units.round1(min(100.0f, (blockedWidth / roadWidthAtTarget) * 100.0f))

                // Traversable corridors on left and right sides of this obstacle
                val clearLeft = Units.round2(max(0.0f, interLeft - roadLeftX))
                val clearRight = Units.round2(max(0.0f, roadRightX - interRight))
                val maxCorridor = max(clearLeft, clearRight)

                if (blockagePct > 10.0f) {
                    blockingIds.add(target.id)
                }

                if (blockagePct > worstBlockagePct) {
                    worstBlockagePct = blockagePct
                    minClearance = maxCorridor
                    primaryBlockerDist = dist
                    minLeftClearance = clearLeft
                    minRightClearance = clearRight
                }

                updatedObjects.add(
                    target.copy(
                        isOnRoad = true,
                        blockageContributionPct = blockagePct
                    )
                )
            } else {
                // Object is on shoulder / outside road bounds (e.g. worker on berm)
                updatedObjects.add(
                    target.copy(
                        isOnRoad = false,
                        blockageContributionPct = 0.0f
                    )
                )
            }
        }

        val effectiveClearance = if (minClearance == Float.MAX_VALUE) road.drivableWidthAtRoverM * 0.5f else minClearance
        val effectiveLeft = if (minLeftClearance == Float.MAX_VALUE) road.drivableWidthAtRoverM * 0.5f else minLeftClearance
        val effectiveRight = if (minRightClearance == Float.MAX_VALUE) road.drivableWidthAtRoverM * 0.5f else minRightClearance
        val requiredPassageWidth = roverWidthM + safetyMarginM
        val isPassable = effectiveClearance >= requiredPassageWidth

        val severity = when {
            worstBlockagePct <= 5.0f -> RoadBlockageSeverity.NONE
            worstBlockagePct <= 25.0f -> RoadBlockageSeverity.LOW
            worstBlockagePct <= 55.0f -> RoadBlockageSeverity.MODERATE
            worstBlockagePct <= 80.0f -> RoadBlockageSeverity.SEVERE
            else -> RoadBlockageSeverity.TOTAL
        }

        val blockage = RoadBlockage(
            isBlocked = worstBlockagePct > 15.0f,
            blockagePct = worstBlockagePct,
            severity = severity,
            blockingObjectIds = blockingIds,
            traversableCorridorLeftM = effectiveLeft,
            traversableCorridorRightM = effectiveRight,
            maxClearanceM = effectiveClearance,
            isPassable = isPassable,
            estimatedDistanceM = primaryBlockerDist,
            confidence = road.roadConfidence,
        )

        return blockage to updatedObjects
    }

    /**
     * Converts a [RoadBlockage] and [RoadSegmentationResult] into a [RoadGeometryState] for the UI
     * and the wire.
     *
     * The road-validity check comes **first** in the status decision. It used to come third,
     * behind `!blockage.isPassable` and `blockage.isBlocked`, which meant a run with no road
     * perception at all reported `BLOCKED` — the fail-safe [RoadBlockage] defaults are
     * `isPassable = false`, and the previous defaults reported `CLEAR`. Neither is an
     * assessment; the honest answer when nothing perceived a road is
     * [RoadStatus.UNKNOWN], which carries `wireValue = null` and so is omitted from the
     * packet rather than presented to the control room as a measurement.
     */
    fun toRoadGeometryState(
        road: RoadSegmentationResult,
        blockage: RoadBlockage,
    ): RoadGeometryState {
        if (!road.hasValidRoad) {
            // `isSupported = false` and every numeric field at its zero default. Read sites and
            // the packet builder both branch on that flag; see [RoadGeometryState].
            return RoadGeometryState(timestampMs = System.currentTimeMillis())
        }

        val roadStatus = when {
            !blockage.isPassable || blockage.severity == RoadBlockageSeverity.TOTAL -> RoadStatus.BLOCKED
            blockage.isBlocked -> RoadStatus.PARTIALLY_BLOCKED
            else -> RoadStatus.CLEAR
        }

        return RoadGeometryState(
            isSupported = true,
            drivableWidthAtRoverM = road.drivableWidthAtRoverM,
            drivableWidthAtObstacleM = road.getDrivableWidthAtDistance(blockage.estimatedDistanceM),
            roadBlockagePct = blockage.blockagePct,
            traversableCorridorLeftM = blockage.traversableCorridorLeftM,
            traversableCorridorRightM = blockage.traversableCorridorRightM,
            maxTraversableCorridorM = blockage.maxClearanceM,
            isPassable = blockage.isPassable,
            roadStatus = roadStatus,
            leftBoundaryBearingDeg = road.leftBoundaryBearingDeg,
            rightBoundaryBearingDeg = road.rightBoundaryBearingDeg,
            roadCenterlineOffsetDeg = road.roadCenterlineOffsetDeg,
            primaryObstacleId = blockage.blockingObjectIds.firstOrNull(),
            timestampMs = System.currentTimeMillis(),
        )
    }
}
