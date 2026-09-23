package com.minesafety.roboeye.ui.components

import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.clickable
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.layout.width
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.foundation.shape.RoundedCornerShape
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
import com.minesafety.roboeye.fusion.FreeSpaceClearance
import com.minesafety.roboeye.fusion.FusedFreeSpace
import com.minesafety.roboeye.localization.LocalPose
import com.minesafety.roboeye.localization.MonocularScaleState
import com.minesafety.roboeye.localization.TrackingQuality
import com.minesafety.roboeye.ui.theme.RadarAmber
import com.minesafety.roboeye.ui.theme.RadarCyan
import com.minesafety.roboeye.ui.theme.RadarGreen
import com.minesafety.roboeye.ui.theme.RadarRed
import com.minesafety.roboeye.ui.theme.TextMuted
import com.minesafety.roboeye.ui.theme.TextPrimary
import com.minesafety.roboeye.vision.MotionState
import com.minesafety.roboeye.vision.RecommendedCorridor
import java.util.Locale

/**
 * Localization Card displaying true [LocalPose] tracking state, relative position,
 * confidence bar, and monocular scale status.
 */
@Composable
fun LocalizationCard(
    localPose: LocalPose?,
    modifier: Modifier = Modifier,
) {
    val quality = localPose?.trackingState ?: TrackingQuality.INITIALIZING
    val scaleState = localPose?.scaleState ?: MonocularScaleState.SCALE_UNKNOWN
    val conf = localPose?.confidence ?: 0.0f
    val isMetric = scaleState == MonocularScaleState.SCALE_ESTIMATED

    val (qualityColor, qualityLabel) = when (quality) {
        TrackingQuality.TRACKING -> RadarGreen to "TRACKING"
        TrackingQuality.DEGRADED -> RadarAmber to "DEGRADED"
        TrackingQuality.LOST -> RadarRed to "LOST"
        TrackingQuality.RELOCALIZING -> RadarCyan to "RELOCALIZING"
        TrackingQuality.INITIALIZING -> TextMuted to "INITIALIZING"
    }

    val (scaleColor, scaleLabel) = when (scaleState) {
        MonocularScaleState.SCALE_ESTIMATED -> RadarGreen to "ESTIMATED"
        MonocularScaleState.SCALE_UNRELIABLE -> RadarAmber to "UNRELIABLE"
        MonocularScaleState.SCALE_UNKNOWN -> TextMuted to "UNKNOWN"
    }

    Column(
        modifier = modifier
            .clip(RoundedCornerShape(6.dp))
            .background(Color(0xFF0F172A))
            .border(1.dp, qualityColor.copy(alpha = 0.5f), RoundedCornerShape(6.dp))
            .padding(8.dp),
        verticalArrangement = Arrangement.spacedBy(4.dp)
    ) {
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
                        .background(qualityColor)
                )
                Spacer(modifier = Modifier.width(4.dp))
                Text(
                    text = "LOCALIZATION",
                    color = TextMuted,
                    fontSize = 9.sp,
                    fontFamily = FontFamily.Monospace,
                    fontWeight = FontWeight.Bold,
                )
            }
            Text(
                text = "● $qualityLabel",
                color = qualityColor,
                fontSize = 10.sp,
                fontFamily = FontFamily.Monospace,
                fontWeight = FontWeight.Bold,
            )
        }

        // Warning banner for LOST or RELOCALIZING states
        if (quality == TrackingQuality.LOST) {
            Box(
                modifier = Modifier
                    .fillMaxWidth()
                    .clip(RoundedCornerShape(3.dp))
                    .background(RadarRed.copy(alpha = 0.25f))
                    .padding(horizontal = 6.dp, vertical = 2.dp)
            ) {
                Text(
                    text = "⚠ LOCALIZATION LOST • MAP UNCONFIRMED",
                    color = RadarRed,
                    fontSize = 8.sp,
                    fontWeight = FontWeight.Bold,
                    fontFamily = FontFamily.Monospace,
                )
            }
        } else if (quality == TrackingQuality.RELOCALIZING) {
            Box(
                modifier = Modifier
                    .fillMaxWidth()
                    .clip(RoundedCornerShape(3.dp))
                    .background(RadarCyan.copy(alpha = 0.25f))
                    .padding(horizontal = 6.dp, vertical = 2.dp)
            ) {
                Text(
                    text = "⟳ MATCHING KEYFRAME LANDMARKS...",
                    color = RadarCyan,
                    fontSize = 8.sp,
                    fontWeight = FontWeight.Bold,
                    fontFamily = FontFamily.Monospace,
                )
            }
        }

        // Relative coordinates
        Row(
            modifier = Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.SpaceBetween,
            verticalAlignment = Alignment.CenterVertically,
        ) {
            val xLabel = if (isMetric) "X (m)" else "REL X"
            val yLabel = if (isMetric) "Y (m)" else "REL Y"
            val xVal = localPose?.let { String.format(Locale.US, "%+.2f", it.xM) } ?: "—"
            val yVal = localPose?.let { String.format(Locale.US, "%+.2f", it.yM) } ?: "—"
            val yawVal = localPose?.let { String.format(Locale.US, "%+.1f°", it.yawDeg) } ?: "—"

            Column {
                Text(xLabel, color = TextMuted, fontSize = 8.sp, fontFamily = FontFamily.Monospace)
                Text(xVal, color = TextPrimary, fontSize = 12.sp, fontWeight = FontWeight.Bold, fontFamily = FontFamily.Monospace)
            }
            Column {
                Text(yLabel, color = TextMuted, fontSize = 8.sp, fontFamily = FontFamily.Monospace)
                Text(yVal, color = TextPrimary, fontSize = 12.sp, fontWeight = FontWeight.Bold, fontFamily = FontFamily.Monospace)
            }
            Column {
                Text("YAW", color = TextMuted, fontSize = 8.sp, fontFamily = FontFamily.Monospace)
                Text(yawVal, color = RadarCyan, fontSize = 12.sp, fontWeight = FontWeight.Bold, fontFamily = FontFamily.Monospace)
            }
            Column(horizontalAlignment = Alignment.End) {
                Text("SCALE", color = TextMuted, fontSize = 8.sp, fontFamily = FontFamily.Monospace)
                Text(scaleLabel, color = scaleColor, fontSize = 10.sp, fontWeight = FontWeight.Bold, fontFamily = FontFamily.Monospace)
            }
        }

        // Confidence progress bar
        Row(
            modifier = Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.SpaceBetween,
            verticalAlignment = Alignment.CenterVertically,
        ) {
            Text("CONFIDENCE", color = TextMuted, fontSize = 8.sp, fontFamily = FontFamily.Monospace)
            Text("${(conf * 100).toInt()}%", color = qualityColor, fontSize = 9.sp, fontWeight = FontWeight.Bold, fontFamily = FontFamily.Monospace)
        }
        LinearProgressIndicator(
            progress = { conf },
            modifier = Modifier
                .fillMaxWidth()
                .height(3.dp)
                .clip(RoundedCornerShape(1.dp)),
            color = qualityColor,
            trackColor = Color(0xFF1E293B),
        )
    }
}

/**
 * Motion State Card displaying macroscopic [MotionState] and sensor source.
 */
@Composable
fun MotionCard(
    motionState: MotionState,
    motionConfidence: Float,
    isImuAvailable: Boolean,
    modifier: Modifier = Modifier,
) {
    val stateColor = when (motionState) {
        MotionState.FORWARD_TRANSLATION -> RadarGreen
        MotionState.TURNING_LEFT, MotionState.TURNING_RIGHT -> RadarCyan
        MotionState.COMBINED_MOTION -> Color(0xFF00E5FF)
        MotionState.STATIONARY -> RadarAmber
        MotionState.BACKWARD_TRANSLATION -> Color(0xFFCE93D8)
        MotionState.IMU_ONLY -> Color(0xFFFFB300)
        MotionState.VISUAL_TRACKING_LOST, MotionState.UNKNOWN -> RadarRed
    }

    val sensorSource = when {
        isImuAvailable && motionState != MotionState.UNKNOWN -> "VISUAL + IMU"
        isImuAvailable -> "IMU ONLY"
        motionState != MotionState.UNKNOWN -> "VISUAL ONLY"
        else -> "NO SOURCE"
    }

    Column(
        modifier = modifier
            .clip(RoundedCornerShape(6.dp))
            .background(Color(0xFF0F172A))
            .border(1.dp, stateColor.copy(alpha = 0.5f), RoundedCornerShape(6.dp))
            .padding(8.dp),
        verticalArrangement = Arrangement.spacedBy(4.dp)
    ) {
        Row(
            modifier = Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.SpaceBetween,
            verticalAlignment = Alignment.CenterVertically,
        ) {
            Text("MOTION", color = TextMuted, fontSize = 9.sp, fontFamily = FontFamily.Monospace, fontWeight = FontWeight.Bold)
            Text(sensorSource, color = Color(0xFF94A3B8), fontSize = 8.sp, fontFamily = FontFamily.Monospace)
        }

        Row(
            modifier = Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.SpaceBetween,
            verticalAlignment = Alignment.CenterVertically,
        ) {
            Text(
                text = motionState.name.replace("_", " "),
                color = stateColor,
                fontSize = 12.sp,
                fontWeight = FontWeight.Bold,
                fontFamily = FontFamily.Monospace,
            )
            Text(
                text = "${(motionConfidence * 100).toInt()}%",
                color = stateColor,
                fontSize = 10.sp,
                fontWeight = FontWeight.Bold,
                fontFamily = FontFamily.Monospace,
            )
        }

        LinearProgressIndicator(
            progress = { motionConfidence },
            modifier = Modifier
                .fillMaxWidth()
                .height(3.dp)
                .clip(RoundedCornerShape(1.dp)),
            color = stateColor,
            trackColor = Color(0xFF1E293B),
        )
    }
}

/**
 * Free-Space Card displaying Phase 4 3-corridor traversability and recommended route.
 * Strictly respects the "Unknown Is NOT Safe" principle.
 */
@Composable
fun FreeSpaceCard(
    freeSpace: FusedFreeSpace?,
    recommendedCorridor: RecommendedCorridor,
    modifier: Modifier = Modifier,
) {
    val detourColor = when (recommendedCorridor) {
        RecommendedCorridor.CENTER -> RadarGreen
        RecommendedCorridor.LEFT_DETOUR, RecommendedCorridor.RIGHT_DETOUR -> RadarCyan
        RecommendedCorridor.NONE_AVAILABLE -> RadarRed
    }

    Column(
        modifier = modifier
            .clip(RoundedCornerShape(6.dp))
            .background(Color(0xFF0F172A))
            .border(1.dp, detourColor.copy(alpha = 0.5f), RoundedCornerShape(6.dp))
            .padding(8.dp),
        verticalArrangement = Arrangement.spacedBy(4.dp)
    ) {
        Row(
            modifier = Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.SpaceBetween,
            verticalAlignment = Alignment.CenterVertically,
        ) {
            Text("FREE SPACE", color = TextMuted, fontSize = 9.sp, fontFamily = FontFamily.Monospace, fontWeight = FontWeight.Bold)
            Text(
                text = "ROUTE: ${recommendedCorridor.name.replace("_", " ")}",
                color = detourColor,
                fontSize = 9.sp,
                fontWeight = FontWeight.Bold,
                fontFamily = FontFamily.Monospace,
            )
        }

        Row(
            modifier = Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.spacedBy(4.dp),
        ) {
            CorridorPill(
                label = "LEFT",
                distM = freeSpace?.left?.clearDistanceM ?: 0f,
                clearance = freeSpace?.left?.clearance ?: FreeSpaceClearance.UNKNOWN,
                modifier = Modifier.weight(1f),
            )
            CorridorPill(
                label = "CENTER",
                distM = freeSpace?.center?.clearDistanceM ?: 0f,
                clearance = freeSpace?.center?.clearance ?: FreeSpaceClearance.UNKNOWN,
                modifier = Modifier.weight(1.3f),
            )
            CorridorPill(
                label = "RIGHT",
                distM = freeSpace?.right?.clearDistanceM ?: 0f,
                clearance = freeSpace?.right?.clearance ?: FreeSpaceClearance.UNKNOWN,
                modifier = Modifier.weight(1f),
            )
        }
    }
}

@Composable
private fun CorridorPill(
    label: String,
    distM: Float,
    clearance: FreeSpaceClearance,
    modifier: Modifier = Modifier,
) {
    val (color, text) = when (clearance) {
        FreeSpaceClearance.CLEAR_WITH_CONFIDENCE -> RadarGreen to "${String.format(Locale.US, "%.1f", distM)}m"
        FreeSpaceClearance.BLOCKED -> RadarRed to "BLOCKED"
        FreeSpaceClearance.CONFLICTING -> RadarAmber to "UNCERTAIN"
        FreeSpaceClearance.UNKNOWN -> TextMuted to "UNKNOWN"
    }

    Box(
        modifier = modifier
            .clip(RoundedCornerShape(3.dp))
            .background(color.copy(alpha = 0.15f))
            .border(1.dp, color.copy(alpha = 0.5f), RoundedCornerShape(3.dp))
            .padding(vertical = 3.dp),
        contentAlignment = Alignment.Center,
    ) {
        Column(horizontalAlignment = Alignment.CenterHorizontally) {
            Text(label, color = TextMuted, fontSize = 7.sp, fontFamily = FontFamily.Monospace)
            Text(text, color = color, fontSize = 9.sp, fontWeight = FontWeight.Bold, fontFamily = FontFamily.Monospace)
        }
    }
}

/**
 * Phase 6 Autonomous Navigation & Control Card.
 * Exposes real navigation state, goal, path distance, final command, and operator controls.
 */
@Composable
fun AutonomousNavigationCard(
    navigationState: com.minesafety.roboeye.nav.state.NavigationStateSnapshot?,
    activeGoal: com.minesafety.roboeye.nav.model.LocalGoal?,
    activePath: com.minesafety.roboeye.nav.planner.PlannedPath?,
    lastCommand: com.minesafety.roboeye.nav.arbiter.ArbitratedCommand?,
    onSetSampleGoal: () -> Unit = {},
    onClearGoal: () -> Unit = {},
    onEmergencyStop: () -> Unit = {},
    onClearEmergencyStop: () -> Unit = {},
    modifier: Modifier = Modifier,
) {
    val state = navigationState?.state ?: com.minesafety.roboeye.nav.state.NavigationState.IDLE
    val (stateColor, stateLabel) = when (state) {
        com.minesafety.roboeye.nav.state.NavigationState.NAVIGATING -> RadarGreen to "NAVIGATING"
        com.minesafety.roboeye.nav.state.NavigationState.READY -> RadarCyan to "READY"
        com.minesafety.roboeye.nav.state.NavigationState.SLOWING,
        com.minesafety.roboeye.nav.state.NavigationState.AVOIDING -> RadarAmber to state.label
        com.minesafety.roboeye.nav.state.NavigationState.STOPPING,
        com.minesafety.roboeye.nav.state.NavigationState.GOAL_REACHED -> RadarCyan to state.label
        com.minesafety.roboeye.nav.state.NavigationState.ESTOP,
        com.minesafety.roboeye.nav.state.NavigationState.FAULT,
        com.minesafety.roboeye.nav.state.NavigationState.LOCALIZATION_LOST,
        com.minesafety.roboeye.nav.state.NavigationState.LINK_LOST -> RadarRed to state.label
        else -> TextMuted to state.label
    }

    val isEstop = state == com.minesafety.roboeye.nav.state.NavigationState.ESTOP

    Column(
        modifier = modifier
            .fillMaxWidth()
            .clip(RoundedCornerShape(6.dp))
            .background(Color(0xFF0F172A))
            .border(1.dp, stateColor.copy(alpha = 0.5f), RoundedCornerShape(6.dp))
            .padding(8.dp),
        verticalArrangement = Arrangement.spacedBy(5.dp)
    ) {
        // Header: Navigation State & Mode
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
                        .background(stateColor)
                )
                Spacer(modifier = Modifier.width(4.dp))
                Text(
                    text = "AUTONOMOUS NAV",
                    color = TextMuted,
                    fontSize = 9.sp,
                    fontFamily = FontFamily.Monospace,
                    fontWeight = FontWeight.Bold,
                )
            }
            Text(
                text = stateLabel,
                color = stateColor,
                fontSize = 9.sp,
                fontWeight = FontWeight.Bold,
                fontFamily = FontFamily.Monospace,
            )
        }

        // Goal & Path telemetry row
        Row(
            modifier = Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.SpaceBetween,
            verticalAlignment = Alignment.CenterVertically,
        ) {
            val goalStr = if (activeGoal != null && activeGoal.valid) {
                String.format(Locale.US, "GOAL: (%+.1f, %+.1f)m", activeGoal.targetX, activeGoal.targetY)
            } else "GOAL: NONE"
            val pathStr = if (activePath != null && !activePath.isEmpty) {
                String.format(Locale.US, "PATH: %.1fm (%d wp)", activePath.totalLengthM, activePath.size)
            } else "PATH: NONE"

            Text(goalStr, color = RadarCyan, fontSize = 9.sp, fontFamily = FontFamily.Monospace)
            Text(pathStr, color = TextPrimary, fontSize = 9.sp, fontFamily = FontFamily.Monospace)
        }

        // Last command & Reason row
        if (lastCommand != null) {
            Row(
                modifier = Modifier.fillMaxWidth(),
                horizontalArrangement = Arrangement.SpaceBetween,
                verticalAlignment = Alignment.CenterVertically,
            ) {
                val cmd = lastCommand.command
                val cmdText = String.format(Locale.US, "CMD: V=%.2f W=%.2f", cmd.linearVelocityMps, cmd.angularVelocityRadS)
                Text(cmdText, color = if (cmd.emergencyStop) RadarRed else TextPrimary, fontSize = 8.sp, fontFamily = FontFamily.Monospace)
                Text(lastCommand.reason.take(24), color = TextMuted, fontSize = 8.sp, fontFamily = FontFamily.Monospace)
            }
        }

        // Quick Operator Action Buttons
        Row(
            modifier = Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.spacedBy(4.dp),
        ) {
            if (activeGoal == null || !activeGoal.valid) {
                Box(
                    modifier = Modifier
                        .weight(1f)
                        .clip(RoundedCornerShape(3.dp))
                        .background(RadarCyan.copy(alpha = 0.2f))
                        .border(1.dp, RadarCyan.copy(alpha = 0.6f), RoundedCornerShape(3.dp))
                        .clickable { onSetSampleGoal() }
                        .padding(vertical = 3.dp),
                    contentAlignment = Alignment.Center,
                ) {
                    Text("+1.5m FWD GOAL", color = RadarCyan, fontSize = 8.sp, fontWeight = FontWeight.Bold, fontFamily = FontFamily.Monospace)
                }
            } else {
                Box(
                    modifier = Modifier
                        .weight(1f)
                        .clip(RoundedCornerShape(3.dp))
                        .background(Color(0xFF334155))
                        .border(1.dp, Color(0xFF64748B), RoundedCornerShape(3.dp))
                        .clickable { onClearGoal() }
                        .padding(vertical = 3.dp),
                    contentAlignment = Alignment.Center,
                ) {
                    Text("CLEAR GOAL", color = Color.White, fontSize = 8.sp, fontWeight = FontWeight.Bold, fontFamily = FontFamily.Monospace)
                }
            }

            // E-STOP Button
            if (!isEstop) {
                Box(
                    modifier = Modifier
                        .weight(1f)
                        .clip(RoundedCornerShape(3.dp))
                        .background(RadarRed.copy(alpha = 0.3f))
                        .border(1.dp, RadarRed, RoundedCornerShape(3.dp))
                        .clickable { onEmergencyStop() }
                        .padding(vertical = 3.dp),
                    contentAlignment = Alignment.Center,
                ) {
                    Text("E-STOP", color = RadarRed, fontSize = 8.sp, fontWeight = FontWeight.Bold, fontFamily = FontFamily.Monospace)
                }
            } else {
                Box(
                    modifier = Modifier
                        .weight(1f)
                        .clip(RoundedCornerShape(3.dp))
                        .background(RadarGreen.copy(alpha = 0.3f))
                        .border(1.dp, RadarGreen, RoundedCornerShape(3.dp))
                        .clickable { onClearEmergencyStop() }
                        .padding(vertical = 3.dp),
                    contentAlignment = Alignment.Center,
                ) {
                    Text("CLEAR E-STOP", color = RadarGreen, fontSize = 8.sp, fontWeight = FontWeight.Bold, fontFamily = FontFamily.Monospace)
                }
            }
        }
    }
}
