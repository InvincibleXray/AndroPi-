package com.minesafety.roboeye.fusion

import com.minesafety.roboeye.vision.SectorRisk

/**
 * Common boundary interface for free-space corridor evidence.
 */
sealed interface FreeSpaceEvidence {
    val sector: CorridorSector
    val confidence: Float
    val isFresh: Boolean
}

/**
 * Pure monocular camera + phone IMU visual free-space evidence.
 *
 * Encapsulates spatial corridor evidence derived from:
 * - Regional Time-To-Collision (TTC) and radial FOE expansion
 * - Camera mount geometry and IMU pitch tilt
 * - Valid feature track density in the sector
 */
data class VisualFreeSpaceEvidence(
    override val sector: CorridorSector,
    override val confidence: Float,
    override val isFresh: Boolean,
    val clearanceDistanceM: Float,
    val risk: SectorRisk,
    val featureCount: Int,
    val isDivergent: Boolean,
    val minTtcSec: Float? = null,
) : FreeSpaceEvidence

/**
 * Dormant physical distance sensor evidence for future multi-sensor phases.
 */
data class RangeFreeSpaceEvidence(
    override val sector: CorridorSector,
    override val confidence: Float,
    override val isFresh: Boolean,
    val rangeM: Float?,
) : FreeSpaceEvidence
