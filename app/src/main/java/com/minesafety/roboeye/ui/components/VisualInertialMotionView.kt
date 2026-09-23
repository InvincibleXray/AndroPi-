package com.minesafety.roboeye.ui.components

import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.layout.width
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.foundation.verticalScroll
import androidx.compose.material3.LinearProgressIndicator
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.text.font.FontFamily
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.minesafety.roboeye.core.ImuReading
import com.minesafety.roboeye.fusion.FreeSpaceClearance
import com.minesafety.roboeye.ui.theme.RadarAmber
import com.minesafety.roboeye.ui.theme.RadarCyan
import com.minesafety.roboeye.ui.theme.RadarGreen
import com.minesafety.roboeye.ui.theme.RadarGridColor
import com.minesafety.roboeye.ui.theme.RadarRed
import com.minesafety.roboeye.ui.theme.TextMuted
import com.minesafety.roboeye.nav.model.DistanceCertainty
import com.minesafety.roboeye.nav.model.WorldObject
import com.minesafety.roboeye.vision.GeometricVisionResult
import com.minesafety.roboeye.vision.GeometryTrustLevel
import com.minesafety.roboeye.vision.MotionState
import com.minesafety.roboeye.vision.RecommendedCorridor
import com.minesafety.roboeye.vision.SectorRisk
import com.minesafety.roboeye.vision.VisualImuConsistency
import java.util.Locale

/**
 * Real-time Camera + Phone IMU Visual-Inertial Motion & Geometric Free-Space Panel.
 *
 * Directly replaces legacy synthetic PPI radar. All displayed metrics are derived
 * from real physical IMU telemetry and on-device geometric optical flow tracking:
 * - Deterministic macroscopic Motion State classification
 * - Cross-sensor Visual / IMU Consistency
 * - Kinematic rates: Gyro yaw rate, tilt (pitch/roll), linear acceleration
 * - Sparse feature tracking & RANSAC geometric consensus inliers
 * - Geometric Free-Space 3-Corridor traversability & detour recommendations
 * - Scene Geometry Trustworthiness level
 * - Contextual Semantic Radar layer (Phase 6C)
 */
@Composable
fun VisualInertialMotionView(
    result: GeometricVisionResult?,
    imuReading: ImuReading?,
    fps: Float,
    latencyMs: Long,
    semanticObjects: List<WorldObject> = emptyList(),
    modifier: Modifier = Modifier,
) {
    val state = result?.motionState ?: MotionState.UNKNOWN
    val consistency = result?.consistency ?: VisualImuConsistency.INSUFFICIENT_DATA
    val confidence = result?.motionConfidence ?: 0.0f
    val inlierCount = result?.inlierCount ?: 0
    val detectedCount = result?.detectedFeaturesCount ?: 0
    val trackedCount = result?.trackedFeaturesCount ?: 0
    val yawRate = result?.rotationalDps ?: (imuReading?.yawRateDps ?: 0.0f)
    val pitch = imuReading?.pitchDeg ?: 0.0f
    val roll = imuReading?.rollDeg ?: 0.0f
    val freeSpace = result?.freeSpace
    val geometryTrust = result?.geometryTrust ?: GeometryTrustLevel.UNTRUSTED
    val recommendedCorridor = result?.recommendedCorridor ?: RecommendedCorridor.NONE_AVAILABLE

    val stateColor = when (state) {
        MotionState.FORWARD_TRANSLATION -> RadarGreen
        MotionState.TURNING_LEFT, MotionState.TURNING_RIGHT -> RadarCyan
        MotionState.COMBINED_MOTION -> Color(0xFF00E5FF)
        MotionState.STATIONARY -> RadarAmber
        MotionState.IMU_ONLY -> Color(0xFFFFB300)
        MotionState.VISUAL_TRACKING_LOST, MotionState.UNKNOWN -> RadarRed
        MotionState.BACKWARD_TRANSLATION -> Color(0xFFCE93D8)
    }

    val consistencyColor = when (consistency) {
        VisualImuConsistency.CONSISTENT -> RadarGreen
        VisualImuConsistency.PARTIALLY_CONSISTENT -> RadarAmber
        VisualImuConsistency.CONFLICTING -> RadarRed
        VisualImuConsistency.INSUFFICIENT_DATA -> TextMuted
    }

    val trustColor = when (geometryTrust) {
        GeometryTrustLevel.TRUSTED -> RadarGreen
        GeometryTrustLevel.DEGRADED -> RadarAmber
        GeometryTrustLevel.UNTRUSTED -> RadarRed
    }

    Box(
        modifier = modifier
            .background(Color(0xFF0A0F14))
            .border(1.dp, RadarGridColor.copy(alpha = 0.5f))
            .padding(10.dp)
    ) {
        Column(
            modifier = Modifier
                .fillMaxSize()
                .verticalScroll(rememberScrollState()),
            verticalArrangement = Arrangement.spacedBy(8.dp)
        ) {
            // 1. Header Bar
            Row(
                modifier = Modifier.fillMaxWidth(),
                horizontalArrangement = Arrangement.SpaceBetween,
                verticalAlignment = Alignment.CenterVertically,
            ) {
                Row(verticalAlignment = Alignment.CenterVertically) {
                    Box(
                        modifier = Modifier
                            .size(8.dp)
                            .clip(CircleShape)
                            .background(if (result != null) RadarGreen else RadarRed)
                    )
                    Spacer(modifier = Modifier.width(6.dp))
                    Text(
                        text = "ENGINEERING DIAGNOSTICS",
                        color = Color.White,
                        fontSize = 11.sp,
                        fontWeight = FontWeight.Bold,
                        fontFamily = FontFamily.Monospace,
                        letterSpacing = 1.sp,
                    )
                }
                Text(
                    text = "${String.format(Locale.US, "%.1f", fps)} FPS | ${latencyMs}ms",
                    color = TextMuted,
                    fontSize = 10.sp,
                    fontFamily = FontFamily.Monospace,
                )
            }

            // 2. Hero Motion State Card
            Box(
                modifier = Modifier
                    .fillMaxWidth()
                    .clip(RoundedCornerShape(6.dp))
                    .background(Color(0xFF101921))
                    .border(1.dp, stateColor.copy(alpha = 0.7f), RoundedCornerShape(6.dp))
                    .padding(8.dp)
            ) {
                Column {
                    Row(
                        modifier = Modifier.fillMaxWidth(),
                        horizontalArrangement = Arrangement.SpaceBetween,
                        verticalAlignment = Alignment.CenterVertically,
                    ) {
                        Text(
                            text = "ESTIMATED STATE",
                            color = TextMuted,
                            fontSize = 9.sp,
                            fontFamily = FontFamily.Monospace,
                        )
                        Text(
                            text = "${(confidence * 100).toInt()}% CONFIDENCE",
                            color = stateColor,
                            fontSize = 10.sp,
                            fontWeight = FontWeight.Bold,
                            fontFamily = FontFamily.Monospace,
                        )
                    }
                    Spacer(modifier = Modifier.height(4.dp))
                    Text(
                        text = state.name.replace("_", " "),
                        color = stateColor,
                        fontSize = 15.sp,
                        fontWeight = FontWeight.Black,
                        fontFamily = FontFamily.Monospace,
                    )
                    Spacer(modifier = Modifier.height(4.dp))
                    LinearProgressIndicator(
                        progress = { confidence },
                        modifier = Modifier
                            .fillMaxWidth()
                            .height(4.dp)
                            .clip(RoundedCornerShape(2.dp)),
                        color = stateColor,
                        trackColor = Color(0xFF1E293B),
                    )
                }
            }

            // 3. Sensor Consistency & Geometry Trust Chips
            Row(
                modifier = Modifier.fillMaxWidth(),
                horizontalArrangement = Arrangement.spacedBy(6.dp),
            ) {
                Box(
                    modifier = Modifier
                        .weight(1f)
                        .clip(RoundedCornerShape(4.dp))
                        .background(Color(0xFF141E28))
                        .border(1.dp, consistencyColor.copy(alpha = 0.4f), RoundedCornerShape(4.dp))
                        .padding(6.dp)
                ) {
                    Column {
                        Text(
                            text = "CAM/IMU SYNC",
                            color = TextMuted,
                            fontSize = 8.sp,
                            fontFamily = FontFamily.Monospace,
                        )
                        Spacer(modifier = Modifier.height(2.dp))
                        Text(
                            text = consistency.name.replace("_", " "),
                            color = consistencyColor,
                            fontSize = 10.sp,
                            fontWeight = FontWeight.Bold,
                            fontFamily = FontFamily.Monospace,
                        )
                    }
                }

                Box(
                    modifier = Modifier
                        .weight(1f)
                        .clip(RoundedCornerShape(4.dp))
                        .background(Color(0xFF141E28))
                        .border(1.dp, trustColor.copy(alpha = 0.4f), RoundedCornerShape(4.dp))
                        .padding(6.dp)
                ) {
                    Column {
                        Text(
                            text = "GEOMETRY TRUST",
                            color = TextMuted,
                            fontSize = 8.sp,
                            fontFamily = FontFamily.Monospace,
                        )
                        Spacer(modifier = Modifier.height(2.dp))
                        Text(
                            text = geometryTrust.name,
                            color = trustColor,
                            fontSize = 10.sp,
                            fontWeight = FontWeight.Bold,
                            fontFamily = FontFamily.Monospace,
                        )
                    }
                }
            }

            // 4. Kinematics Grid (Gyro & Feature Inliers)
            Row(
                modifier = Modifier.fillMaxWidth(),
                horizontalArrangement = Arrangement.spacedBy(6.dp),
            ) {
                Box(
                    modifier = Modifier
                        .weight(1f)
                        .clip(RoundedCornerShape(4.dp))
                        .background(Color(0xFF141E28))
                        .padding(6.dp)
                ) {
                    Column {
                        Text(
                            text = "GYRO / TILT",
                            color = TextMuted,
                            fontSize = 8.sp,
                            fontFamily = FontFamily.Monospace,
                        )
                        Spacer(modifier = Modifier.height(2.dp))
                        Text(
                            text = "${String.format(Locale.US, "%+.0f", yawRate)}°/s | ${pitch.toInt()}°",
                            color = Color.White,
                            fontSize = 11.sp,
                            fontWeight = FontWeight.SemiBold,
                            fontFamily = FontFamily.Monospace,
                        )
                    }
                }

                Box(
                    modifier = Modifier
                        .weight(1f)
                        .clip(RoundedCornerShape(4.dp))
                        .background(Color(0xFF141E28))
                        .padding(6.dp)
                ) {
                    Column {
                        Text(
                            text = "RANSAC INLIERS",
                            color = TextMuted,
                            fontSize = 8.sp,
                            fontFamily = FontFamily.Monospace,
                        )
                        Spacer(modifier = Modifier.height(2.dp))
                        Text(
                            text = "$inlierCount / $trackedCount ($detectedCount)",
                            color = if (inlierCount >= 10) RadarGreen else RadarRed,
                            fontSize = 11.sp,
                            fontWeight = FontWeight.SemiBold,
                            fontFamily = FontFamily.Monospace,
                        )
                    }
                }
            }

            // 5. Phase 4: Free-Space Traversability & Recommended Route
            if (freeSpace != null) {
                val detourColor = when (recommendedCorridor) {
                    RecommendedCorridor.CENTER -> RadarGreen
                    RecommendedCorridor.LEFT_DETOUR, RecommendedCorridor.RIGHT_DETOUR -> RadarCyan
                    RecommendedCorridor.NONE_AVAILABLE -> RadarRed
                }

                Column(
                    modifier = Modifier
                        .fillMaxWidth()
                        .clip(RoundedCornerShape(4.dp))
                        .background(Color(0xFF101921))
                        .border(1.dp, detourColor.copy(alpha = 0.5f), RoundedCornerShape(4.dp))
                        .padding(6.dp)
                ) {
                    Row(
                        modifier = Modifier.fillMaxWidth(),
                        horizontalArrangement = Arrangement.SpaceBetween,
                        verticalAlignment = Alignment.CenterVertically,
                    ) {
                        Text(
                            text = "TRAVERSABLE ROUTE",
                            color = TextMuted,
                            fontSize = 8.sp,
                            fontFamily = FontFamily.Monospace,
                        )
                        Text(
                            text = recommendedCorridor.name.replace("_", " "),
                            color = detourColor,
                            fontSize = 9.sp,
                            fontWeight = FontWeight.Bold,
                            fontFamily = FontFamily.Monospace,
                        )
                    }
                    Spacer(modifier = Modifier.height(4.dp))
                    Row(
                        modifier = Modifier.fillMaxWidth(),
                        horizontalArrangement = Arrangement.spacedBy(4.dp),
                    ) {
                        CorridorClearanceBar(
                            label = "L",
                            distM = freeSpace.left.clearDistanceM,
                            clearance = freeSpace.left.clearance,
                            modifier = Modifier.weight(1f),
                        )
                        CorridorClearanceBar(
                            label = "CTR",
                            distM = freeSpace.center.clearDistanceM,
                            clearance = freeSpace.center.clearance,
                            modifier = Modifier.weight(1.4f),
                        )
                        CorridorClearanceBar(
                            label = "R",
                            distM = freeSpace.right.clearDistanceM,
                            clearance = freeSpace.right.clearance,
                            modifier = Modifier.weight(1f),
                        )
                    }
                }
            }

            // 6. Regional Time-To-Collision
            val ttc = result?.ttcResult
            if (ttc != null) {
                Column(
                    modifier = Modifier
                        .fillMaxWidth()
                        .clip(RoundedCornerShape(4.dp))
                        .background(Color(0xFF101921))
                        .padding(6.dp)
                ) {
                    Row(
                        modifier = Modifier.fillMaxWidth(),
                        horizontalArrangement = Arrangement.SpaceBetween,
                    ) {
                        Text(
                            text = "COLLISION CORRIDORS (TTC)",
                            color = TextMuted,
                            fontSize = 8.sp,
                            fontFamily = FontFamily.Monospace,
                        )
                        val minTtc = ttc.minTtcSec
                        Text(
                            text = if (minTtc != null) "${String.format(Locale.US, "%.1f", minTtc)}s" else "CLEAR",
                            color = if (ttc.overallRisk == SectorRisk.CRITICAL) RadarRed else RadarGreen,
                            fontSize = 9.sp,
                            fontWeight = FontWeight.Bold,
                            fontFamily = FontFamily.Monospace,
                        )
                    }
                    Spacer(modifier = Modifier.height(4.dp))
                    Row(
                        modifier = Modifier.fillMaxWidth(),
                        horizontalArrangement = Arrangement.spacedBy(4.dp),
                    ) {
                        SectorRiskBar(label = "L", risk = ttc.left.risk, modifier = Modifier.weight(1f))
                        SectorRiskBar(label = "CTR", risk = ttc.center.risk, modifier = Modifier.weight(1.5f))
                        SectorRiskBar(label = "R", risk = ttc.right.risk, modifier = Modifier.weight(1f))
                    }
                }
            }

            // 6. Phase 5: Visual-Inertial Localization & Local Mapping HUD
            val localPose = result?.localPose
            val mapState = result?.mapState
            if (localPose != null) {
                val trackingColor = when (localPose.trackingState) {
                    com.minesafety.roboeye.localization.TrackingQuality.TRACKING -> RadarGreen
                    com.minesafety.roboeye.localization.TrackingQuality.DEGRADED -> RadarAmber
                    com.minesafety.roboeye.localization.TrackingQuality.LOST -> RadarRed
                    com.minesafety.roboeye.localization.TrackingQuality.RELOCALIZING -> RadarCyan
                    com.minesafety.roboeye.localization.TrackingQuality.INITIALIZING -> TextMuted
                }

                Column(
                    modifier = Modifier
                        .fillMaxWidth()
                        .clip(RoundedCornerShape(6.dp))
                        .background(Color(0xFF0F1820))
                        .border(1.dp, trackingColor.copy(alpha = 0.5f), RoundedCornerShape(6.dp))
                        .padding(8.dp)
                ) {
                    Row(
                        modifier = Modifier.fillMaxWidth(),
                        horizontalArrangement = Arrangement.SpaceBetween,
                        verticalAlignment = Alignment.CenterVertically,
                    ) {
                        Text(
                            text = "LOCAL POSE (START ORIGIN)",
                            color = TextMuted,
                            fontSize = 8.sp,
                            fontWeight = FontWeight.Bold,
                            fontFamily = FontFamily.Monospace,
                        )
                        Text(
                            text = "[${localPose.trackingState.name}]",
                            color = trackingColor,
                            fontSize = 9.sp,
                            fontWeight = FontWeight.Bold,
                            fontFamily = FontFamily.Monospace,
                        )
                    }
                    Spacer(modifier = Modifier.height(4.dp))
                    Row(
                        modifier = Modifier.fillMaxWidth(),
                        horizontalArrangement = Arrangement.SpaceBetween,
                    ) {
                        Text(
                            text = "X: ${String.format(Locale.US, "%+.2f", localPose.xM)}m",
                            color = Color.White,
                            fontSize = 11.sp,
                            fontWeight = FontWeight.SemiBold,
                            fontFamily = FontFamily.Monospace,
                        )
                        Text(
                            text = "Y: ${String.format(Locale.US, "%+.2f", localPose.yM)}m",
                            color = Color.White,
                            fontSize = 11.sp,
                            fontWeight = FontWeight.SemiBold,
                            fontFamily = FontFamily.Monospace,
                        )
                        Text(
                            text = "HDG: ${String.format(Locale.US, "%+.1f", localPose.yawDeg)}°",
                            color = RadarCyan,
                            fontSize = 11.sp,
                            fontWeight = FontWeight.SemiBold,
                            fontFamily = FontFamily.Monospace,
                        )
                    }
                    if (mapState != null) {
                        Spacer(modifier = Modifier.height(4.dp))
                        Row(
                            modifier = Modifier.fillMaxWidth(),
                            horizontalArrangement = Arrangement.SpaceBetween,
                        ) {
                            Text(
                                text = "MAP: ${mapState.keyframesCount} KF | ${mapState.activeLandmarksCount} LM",
                                color = TextMuted,
                                fontSize = 8.sp,
                                fontFamily = FontFamily.Monospace,
                            )
                            Text(
                                text = "OCC: ${mapState.occupiedCellsCount} | FREE: ${mapState.freeCellsCount}",
                                color = TextMuted,
                                fontSize = 8.sp,
                                fontFamily = FontFamily.Monospace,
                            )
                        }
                    }
                }
            }

            // 7. Semantic Object Radar Card (Phase 6C)
            SemanticRadarCard(semanticObjects = semanticObjects)

            // 8. Diagnostics Footer
            if (result != null) {
                Row(
                    modifier = Modifier.fillMaxWidth(),
                    horizontalArrangement = Arrangement.SpaceBetween,
                ) {
                    Text(
                        text = "FOE: (${result.foeX.toInt()}, ${result.foeY.toInt()})",
                        color = TextMuted,
                        fontSize = 9.sp,
                        fontFamily = FontFamily.Monospace,
                    )
                    Text(
                        text = if (result.isDivergent) "RADIAL EXPANSION" else "FLOW CONTRACTION",
                        color = if (result.isDivergent) RadarGreen else TextMuted,
                        fontSize = 9.sp,
                        fontFamily = FontFamily.Monospace,
                    )
                }
            }
        }
    }
}

@Composable
private fun CorridorClearanceBar(
    label: String,
    distM: Float,
    clearance: FreeSpaceClearance,
    modifier: Modifier = Modifier,
) {
    val (color, text) = when (clearance) {
        FreeSpaceClearance.CLEAR_WITH_CONFIDENCE -> RadarGreen to String.format(Locale.US, "%.1fm", distM)
        FreeSpaceClearance.BLOCKED -> RadarRed to "BLOCKED"
        FreeSpaceClearance.CONFLICTING -> RadarAmber to "CONF"
        FreeSpaceClearance.UNKNOWN -> TextMuted to "UNK"
    }

    Box(
        modifier = modifier
            .clip(RoundedCornerShape(3.dp))
            .background(color.copy(alpha = 0.2f))
            .border(1.dp, color.copy(alpha = 0.6f), RoundedCornerShape(3.dp))
            .padding(vertical = 3.dp),
        contentAlignment = Alignment.Center,
    ) {
        Text(
            text = "$label: $text",
            color = color,
            fontSize = 8.sp,
            fontWeight = FontWeight.Bold,
            fontFamily = FontFamily.Monospace,
        )
    }
}

@Composable
private fun SectorRiskBar(label: String, risk: SectorRisk, modifier: Modifier = Modifier) {
    val (color, text) = when (risk) {
        SectorRisk.SAFE -> RadarGreen to "SAFE"
        SectorRisk.CAUTION -> RadarCyan to "CAUTION"
        SectorRisk.WARNING -> RadarAmber to "WARN"
        SectorRisk.CRITICAL -> RadarRed to "CRIT"
    }

    Box(
        modifier = modifier
            .clip(RoundedCornerShape(3.dp))
            .background(color.copy(alpha = 0.2f))
            .border(1.dp, color.copy(alpha = 0.6f), RoundedCornerShape(3.dp))
            .padding(vertical = 3.dp),
        contentAlignment = Alignment.Center,
    ) {
        Text(
            text = "$label: $text",
            color = color,
            fontSize = 8.sp,
            fontWeight = FontWeight.Bold,
            fontFamily = FontFamily.Monospace,
        )
    }
}

/**
 * Renders contextual tracked semantic objects from [NavigationWorldModel.semanticObjects].
 * Displays active target count, IDs, labels, distance certainty, bearing, coordinates, and threat level.
 */
@Composable
fun SemanticRadarCard(
    semanticObjects: List<WorldObject>,
    modifier: Modifier = Modifier,
) {
    Box(
        modifier = modifier
            .fillMaxWidth()
            .clip(RoundedCornerShape(6.dp))
            .background(Color(0xFF101921))
            .border(1.dp, Color(0xFF1E293B), RoundedCornerShape(6.dp))
            .padding(8.dp)
    ) {
        Column(verticalArrangement = Arrangement.spacedBy(6.dp)) {
            // Header
            Row(
                modifier = Modifier.fillMaxWidth(),
                horizontalArrangement = Arrangement.SpaceBetween,
                verticalAlignment = Alignment.CenterVertically,
            ) {
                Row(verticalAlignment = Alignment.CenterVertically) {
                    Box(
                        modifier = Modifier
                            .size(6.dp)
                            .clip(CircleShape)
                            .background(if (semanticObjects.isNotEmpty()) Color(0xFFE040FB) else TextMuted)
                    )
                    Spacer(modifier = Modifier.width(6.dp))
                    Text(
                        text = "SEMANTIC RADAR",
                        color = Color.White,
                        fontSize = 10.sp,
                        fontWeight = FontWeight.Bold,
                        fontFamily = FontFamily.Monospace,
                        letterSpacing = 0.5.sp,
                    )
                }
                Text(
                    text = "${semanticObjects.size} TARGETS",
                    color = if (semanticObjects.isNotEmpty()) Color(0xFFE040FB) else TextMuted,
                    fontSize = 9.sp,
                    fontWeight = FontWeight.Bold,
                    fontFamily = FontFamily.Monospace,
                )
            }

            if (semanticObjects.isEmpty()) {
                Box(
                    modifier = Modifier
                        .fillMaxWidth()
                        .padding(vertical = 8.dp),
                    contentAlignment = Alignment.Center,
                ) {
                    Text(
                        text = "NO SEMANTIC OBJECTS DETECTED",
                        color = TextMuted,
                        fontSize = 9.sp,
                        fontFamily = FontFamily.Monospace,
                    )
                }
            } else {
                semanticObjects.forEach { obj ->
                    SemanticObjectPill(obj = obj)
                }
            }
        }
    }
}

@Composable
private fun SemanticObjectPill(obj: WorldObject) {
    val threatColor = when {
        obj.threatScore >= 0.7f -> RadarRed
        obj.threatScore >= 0.35f -> RadarAmber
        else -> RadarCyan
    }

    Box(
        modifier = Modifier
            .fillMaxWidth()
            .clip(RoundedCornerShape(4.dp))
            .background(Color(0xFF141E28))
            .border(1.dp, threatColor.copy(alpha = 0.4f), RoundedCornerShape(4.dp))
            .padding(6.dp)
    ) {
        Column(verticalArrangement = Arrangement.spacedBy(3.dp)) {
            // Line 1: ID, Label, Coasting Badge, Threat
            Row(
                modifier = Modifier.fillMaxWidth(),
                horizontalArrangement = Arrangement.SpaceBetween,
                verticalAlignment = Alignment.CenterVertically,
            ) {
                Row(verticalAlignment = Alignment.CenterVertically) {
                    Text(
                        text = "#${obj.trackId} ${obj.label.uppercase(Locale.US)}",
                        color = Color.White,
                        fontSize = 10.sp,
                        fontWeight = FontWeight.Bold,
                        fontFamily = FontFamily.Monospace,
                    )
                    if (obj.isCoasting) {
                        Spacer(modifier = Modifier.width(4.dp))
                        Box(
                            modifier = Modifier
                                .clip(RoundedCornerShape(2.dp))
                                .background(RadarAmber.copy(alpha = 0.2f))
                                .border(1.dp, RadarAmber.copy(alpha = 0.5f), RoundedCornerShape(2.dp))
                                .padding(horizontal = 4.dp, vertical = 1.dp)
                        ) {
                            Text(
                                text = "COASTING",
                                color = RadarAmber,
                                fontSize = 7.sp,
                                fontWeight = FontWeight.Bold,
                                fontFamily = FontFamily.Monospace,
                            )
                        }
                    }
                }

                Text(
                    text = "${(obj.threatScore * 100).toInt()}% THREAT",
                    color = threatColor,
                    fontSize = 8.sp,
                    fontWeight = FontWeight.Bold,
                    fontFamily = FontFamily.Monospace,
                )
            }

            // Line 2: Distance Certainty & Bearing
            Row(
                modifier = Modifier.fillMaxWidth(),
                horizontalArrangement = Arrangement.SpaceBetween,
                verticalAlignment = Alignment.CenterVertically,
            ) {
                val distanceText = if (obj.distanceCertainty == DistanceCertainty.ESTIMATED_PRIOR &&
                    obj.roverDistanceM != null && obj.roverDistanceM.isFinite() && obj.roverDistanceM > 0f) {
                    "Estimated ${String.format(Locale.US, "%.1fm", obj.roverDistanceM)}"
                } else {
                    "Distance: Unknown"
                }

                val distanceColor = if (obj.distanceCertainty == DistanceCertainty.ESTIMATED_PRIOR) {
                    RadarGreen
                } else {
                    TextMuted
                }

                Text(
                    text = distanceText,
                    color = distanceColor,
                    fontSize = 9.sp,
                    fontWeight = FontWeight.SemiBold,
                    fontFamily = FontFamily.Monospace,
                )

                val bearingSign = if (obj.roverBearingDeg >= 0) "+" else ""
                Text(
                    text = "BRG: $bearingSign${String.format(Locale.US, "%.1f°", obj.roverBearingDeg)}",
                    color = RadarCyan,
                    fontSize = 9.sp,
                    fontFamily = FontFamily.Monospace,
                )
            }

            // Line 3: Body Coordinates (X forward, Y lateral) if available
            if (obj.roverXM != null && obj.roverYM != null && obj.roverXM.isFinite() && obj.roverYM.isFinite()) {
                val xSign = if (obj.roverXM >= 0) "+" else ""
                val ySign = if (obj.roverYM >= 0) "+" else ""
                Text(
                    text = "BODY: X: $xSign${String.format(Locale.US, "%.1fm", obj.roverXM)} | Y: $ySign${String.format(Locale.US, "%.1fm", obj.roverYM)}",
                    color = TextMuted,
                    fontSize = 8.sp,
                    fontFamily = FontFamily.Monospace,
                )
            }
        }
    }
}

