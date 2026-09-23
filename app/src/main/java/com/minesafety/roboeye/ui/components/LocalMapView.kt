package com.minesafety.roboeye.ui.components

import androidx.compose.foundation.Canvas
import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.clickable
import androidx.compose.foundation.gestures.detectTapGestures
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.layout.width
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.geometry.Size
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.Path
import androidx.compose.ui.graphics.drawscope.Stroke
import androidx.compose.ui.graphics.drawscope.rotate
import androidx.compose.ui.input.pointer.pointerInput
import androidx.compose.ui.text.font.FontFamily
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.minesafety.roboeye.localization.Landmark
import com.minesafety.roboeye.localization.LocalPose
import com.minesafety.roboeye.localization.TrackingQuality
import com.minesafety.roboeye.mapping.Keyframe
import com.minesafety.roboeye.mapping.LocalSpatialMap
import com.minesafety.roboeye.mapping.MapCellState
import com.minesafety.roboeye.nav.model.DistanceCertainty
import com.minesafety.roboeye.nav.model.WorldObject
import com.minesafety.roboeye.ui.theme.MapAxisColor
import com.minesafety.roboeye.ui.theme.MapBgColor
import com.minesafety.roboeye.ui.theme.MapFreeColor
import com.minesafety.roboeye.ui.theme.MapGridLineColor
import com.minesafety.roboeye.ui.theme.MapLandmarkColor
import com.minesafety.roboeye.ui.theme.MapOccupiedColor
import com.minesafety.roboeye.ui.theme.MapRoverColor
import com.minesafety.roboeye.ui.theme.RadarAmber
import com.minesafety.roboeye.ui.theme.RadarCyan
import com.minesafety.roboeye.ui.theme.RadarRed
import com.minesafety.roboeye.ui.theme.TextMuted
import com.minesafety.roboeye.ui.theme.TextPrimary
import java.util.Locale
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.min
import kotlin.math.sin

/**
 * Tactical 2D Local Spatial Map Display.
 *
 * Visualizes the real Phase 5 [LocalSpatialMap] and [LocalPose] without mock or decorative data:
 * - Fixed 24x24 spatial grid cells (0.25m resolution, 6.0m x 6.0m total area)
 * - True occupancy classification: OCCUPIED (solid obstacle), FREE (clear ground), UNCERTAIN (unknown)
 * - Salient visual-inertial landmarks and historical keyframe origins
 * - Real-time Rover pose arrow and heading beam anchored to start origin
 * - Center-on-Rover vs Fixed-Origin viewport toggles
 * - High performance: zero object allocation in hot draw loop, suitable for Redmi 6A
 * - Contextual Semantic Objects Overlay (Phase 6C)
 */
@Composable
fun LocalMapView(
    spatialMap: LocalSpatialMap,
    localPose: LocalPose,
    landmarks: List<Landmark>,
    keyframes: List<Keyframe>,
    activeGoal: com.minesafety.roboeye.nav.model.LocalGoal? = null,
    activePath: com.minesafety.roboeye.nav.planner.PlannedPath? = null,
    semanticObjects: List<WorldObject> = emptyList(),
    centerOnRover: Boolean = true,
    zoom: Float = 1.0f,
    onToggleCenter: () -> Unit = {},
    onCycleZoom: () -> Unit = {},
    onResetRequest: () -> Unit = {},
    modifier: Modifier = Modifier,
) {
    var selectedCellInfo by remember { mutableStateOf<String?>(null) }

    Box(
        modifier = modifier
            .clip(RoundedCornerShape(8.dp))
            .background(MapBgColor)
            .border(1.dp, Color(0xFF1E293B), RoundedCornerShape(8.dp))
    ) {
        // Main 2D Tactical Canvas
        Canvas(
            modifier = Modifier
                .fillMaxSize()
                .pointerInput(spatialMap, localPose, centerOnRover, zoom, semanticObjects) {
                    detectTapGestures { tapOffset ->
                        val canvasW = size.width.toFloat()
                        val canvasH = size.height.toFloat()
                        val viewSizeM = 6.0f / zoom
                        val scalePxPerM = min(canvasW, canvasH) / viewSizeM

                        val centerXM = if (centerOnRover) localPose.xM else 0.0f
                        val centerYM = if (centerOnRover) localPose.yM else 0.0f

                        val dxPx = tapOffset.x - (canvasW * 0.5f)
                        val dyPx = tapOffset.y - (canvasH * 0.5f)

                        val tapYM = centerYM - (dxPx / scalePxPerM)
                        val tapXM = centerXM - (dyPx / scalePxPerM)

                        // Check if tapped near a semantic object
                        val tappedObj = semanticObjects.firstOrNull { obj ->
                            val objMapX = obj.mapXM
                            val objMapY = obj.mapYM
                            if (objMapX != null && objMapY != null && obj.distanceCertainty != DistanceCertainty.UNKNOWN) {
                                hypot((objMapX - tapXM).toDouble(), (objMapY - tapYM).toDouble()).toFloat() < 0.4f
                            } else false
                        }

                        if (tappedObj != null) {
                            val dStr = if (tappedObj.roverDistanceM != null && tappedObj.distanceCertainty == DistanceCertainty.ESTIMATED_PRIOR) {
                                String.format(Locale.US, "%.1fm", tappedObj.roverDistanceM)
                            } else {
                                "UNK"
                            }
                            selectedCellInfo = "TARGET #${tappedObj.trackId} [${tappedObj.label.uppercase(Locale.US)}] • dist=$dStr • threat=${(tappedObj.threatScore * 100).toInt()}%"
                        } else {
                            val gx = ((tapXM / spatialMap.resolutionM) + (spatialMap.gridWidth / 2)).toInt()
                            val gy = ((tapYM / spatialMap.resolutionM) + (spatialMap.gridHeight / 2)).toInt()

                            if (gx in 0 until spatialMap.gridWidth && gy in 0 until spatialMap.gridHeight) {
                                val cell = spatialMap.cells[gx][gy]
                                val stateName = cell.state.name
                                val confInt = (cell.confidence * 100).toInt()
                                selectedCellInfo = "CELL [$gx,$gy] ${String.format(Locale.US, "X:%+.2fm Y:%+.2fm", cell.mapXM, cell.mapYM)} • $stateName ($confInt%)"
                            } else {
                                selectedCellInfo = null
                            }
                        }
                    }
                }
        ) {
            val canvasW = size.width
            val canvasH = size.height
            val viewSizeM = 6.0f / zoom
            val scalePxPerM = min(canvasW, canvasH) / viewSizeM

            val originScreenX = canvasW * 0.5f
            val originScreenY = canvasH * 0.5f

            val focusXM = if (centerOnRover) localPose.xM else 0.0f
            val focusYM = if (centerOnRover) localPose.yM else 0.0f

            fun mapToScreen(mapXM: Float, mapYM: Float): Offset {
                val relXM = mapXM - focusXM
                val relYM = mapYM - focusYM
                val sx = originScreenX - (relYM * scalePxPerM)
                val sy = originScreenY - (relXM * scalePxPerM)
                return Offset(sx, sy)
            }

            // 1. Draw Background Metric Grid Lines (0.5m & 1.0m intervals)
            val minGridM = -3.0f
            val maxGridM = 3.0f
            var gridLineM = minGridM
            while (gridLineM <= maxGridM) {
                val isMajor = Math.abs(gridLineM % 1.0f) < 0.05f || Math.abs(gridLineM) < 0.05f
                val lineColor = if (Math.abs(gridLineM) < 0.05f) MapAxisColor else MapGridLineColor
                val strokeWidth = if (isMajor) 1.5f else 0.75f

                val pLeft = mapToScreen(gridLineM, minGridM)
                val pRight = mapToScreen(gridLineM, maxGridM)
                drawLine(
                    color = lineColor,
                    start = pLeft,
                    end = pRight,
                    strokeWidth = strokeWidth,
                )

                val pBottom = mapToScreen(minGridM, gridLineM)
                val pTop = mapToScreen(maxGridM, gridLineM)
                drawLine(
                    color = lineColor,
                    start = pBottom,
                    end = pTop,
                    strokeWidth = strokeWidth,
                )

                gridLineM += 0.5f
            }

            // 2. Draw Map Cells from LocalSpatialMap (24x24 fixed array)
            val cellScreenHalfSize = (spatialMap.resolutionM * scalePxPerM * 0.5f)
            val cellSizePx = spatialMap.resolutionM * scalePxPerM

            for (gx in 0 until spatialMap.gridWidth) {
                for (gy in 0 until spatialMap.gridHeight) {
                    val cell = spatialMap.cells[gx][gy]
                    if (cell.state == MapCellState.UNCERTAIN) continue

                    val cellCenter = mapToScreen(cell.mapXM, cell.mapYM)
                    if (cellCenter.x < -cellSizePx || cellCenter.x > canvasW + cellSizePx ||
                        cellCenter.y < -cellSizePx || cellCenter.y > canvasH + cellSizePx) {
                        continue
                    }

                    when (cell.state) {
                        MapCellState.OCCUPIED -> {
                            val alpha = (0.35f + cell.confidence * 0.65f).coerceIn(0f, 1f)
                            drawRect(
                                color = MapOccupiedColor.copy(alpha = alpha),
                                topLeft = Offset(cellCenter.x - cellScreenHalfSize, cellCenter.y - cellScreenHalfSize),
                                size = Size(cellSizePx - 1f, cellSizePx - 1f),
                            )
                        }
                        MapCellState.FREE -> {
                            val alpha = (0.25f + cell.confidence * 0.55f).coerceIn(0f, 1f)
                            drawCircle(
                                color = MapFreeColor.copy(alpha = alpha),
                                radius = (cellSizePx * 0.28f).coerceAtLeast(1.5f),
                                center = cellCenter,
                            )
                        }
                        MapCellState.UNCERTAIN -> { /* Unknown is not drawn */ }
                    }
                }
            }

            // 3. Draw Historical Keyframe Anchors
            for (i in keyframes.indices) {
                val kf = keyframes[i]
                val kfScreen = mapToScreen(kf.pose.xM, kf.pose.yM)
                drawCircle(
                    color = Color(0xFF64748B),
                    radius = 3.5f,
                    center = kfScreen,
                    style = Stroke(width = 1.0f),
                )
            }

            // 4. Draw Active Landmarks
            for (i in landmarks.indices) {
                val lm = landmarks[i]
                val lmScreen = mapToScreen(lm.mapXM, lm.mapYM)
                drawCircle(
                    color = MapLandmarkColor,
                    radius = 2.5f,
                    center = lmScreen,
                )
            }

            // 5. Draw Start Origin Reference Pip (0.0m, 0.0m)
            val originPos = mapToScreen(0.0f, 0.0f)
            drawLine(
                color = RadarCyan.copy(alpha = 0.6f),
                start = Offset(originPos.x - 6f, originPos.y),
                end = Offset(originPos.x + 6f, originPos.y),
                strokeWidth = 1.5f,
            )
            drawLine(
                color = RadarCyan.copy(alpha = 0.6f),
                start = Offset(originPos.x, originPos.y - 6f),
                end = Offset(originPos.x, originPos.y + 6f),
                strokeWidth = 1.5f,
            )

            // 6. Draw Planned Path (Phase 6 Autonomous Route)
            if (activePath != null && activePath.waypoints.size >= 2) {
                for (i in 0 until activePath.waypoints.size - 1) {
                    val p1 = mapToScreen(activePath.waypoints[i].xM, activePath.waypoints[i].yM)
                    val p2 = mapToScreen(activePath.waypoints[i + 1].xM, activePath.waypoints[i + 1].yM)
                    drawLine(
                        color = RadarAmber.copy(alpha = 0.85f),
                        start = p1,
                        end = p2,
                        strokeWidth = 3.0f,
                    )
                    drawCircle(
                        color = RadarAmber,
                        radius = 2.5f,
                        center = p1,
                    )
                }
                val lastP = mapToScreen(activePath.waypoints.last().xM, activePath.waypoints.last().yM)
                drawCircle(color = RadarAmber, radius = 3.0f, center = lastP)
            }

            // 7. Draw Active Navigation Goal
            if (activeGoal != null && activeGoal.valid) {
                val goalScreen = mapToScreen(activeGoal.targetX, activeGoal.targetY)
                val tolerancePx = (activeGoal.positionToleranceM * scalePxPerM).coerceAtLeast(6f)
                drawCircle(
                    color = RadarCyan.copy(alpha = 0.20f),
                    radius = tolerancePx,
                    center = goalScreen,
                )
                drawCircle(
                    color = RadarCyan,
                    radius = tolerancePx,
                    center = goalScreen,
                    style = Stroke(width = 1.5f),
                )
                drawCircle(
                    color = Color.White,
                    radius = 3.5f,
                    center = goalScreen,
                )
            }

            // 8. Draw Rover Ego-Pose (Position & Yaw Direction)
            val roverPos = mapToScreen(localPose.xM, localPose.yM)
            val roverTrackingColor = when (localPose.trackingState) {
                TrackingQuality.TRACKING -> MapRoverColor
                TrackingQuality.DEGRADED -> RadarAmber
                TrackingQuality.LOST -> RadarRed
                TrackingQuality.RELOCALIZING -> Color(0xFF00E5FF)
                TrackingQuality.INITIALIZING -> TextMuted
            }

            val yawRad = Math.toRadians(localPose.yawDeg.toDouble()).toFloat()
            val rayLengthPx = scalePxPerM * 0.9f
            val rayEndX = roverPos.x - sin(yawRad) * rayLengthPx
            val rayEndY = roverPos.y - cos(yawRad) * rayLengthPx

            drawLine(
                color = roverTrackingColor.copy(alpha = 0.7f),
                start = roverPos,
                end = Offset(rayEndX, rayEndY),
                strokeWidth = 2.0f,
            )

            val glyphRadius = 8.0f
            rotate(degrees = -localPose.yawDeg, pivot = roverPos) {
                val path = Path().apply {
                    moveTo(roverPos.x, roverPos.y - glyphRadius * 1.3f)
                    lineTo(roverPos.x - glyphRadius, roverPos.y + glyphRadius * 0.8f)
                    lineTo(roverPos.x, roverPos.y + glyphRadius * 0.3f)
                    lineTo(roverPos.x + glyphRadius, roverPos.y + glyphRadius * 0.8f)
                    close()
                }
                drawPath(path = path, color = roverTrackingColor)
                drawPath(path = path, color = Color.White, style = Stroke(width = 1.0f))
            }

            // 9. Draw Semantic Objects Overlay (Phase 6C)
            if (semanticObjects.isNotEmpty()) {
                semanticObjects.forEach { obj ->
                    val targetMapPos: Pair<Float, Float>? = when {
                        obj.distanceCertainty == DistanceCertainty.UNKNOWN -> null
                        obj.mapXM != null && obj.mapYM != null && obj.mapXM.isFinite() && obj.mapYM.isFinite() -> {
                            Pair(obj.mapXM, obj.mapYM)
                        }
                        obj.roverXM != null && obj.roverYM != null && obj.roverXM.isFinite() && obj.roverYM.isFinite() -> {
                            val yawR = Math.toRadians(localPose.yawDeg.toDouble()).toFloat()
                            val cosY = cos(yawR)
                            val sinY = sin(yawR)
                            val mX = localPose.xM + (obj.roverXM * cosY - obj.roverYM * sinY)
                            val mY = localPose.yM + (obj.roverXM * sinY + obj.roverYM * cosY)
                            Pair(mX, mY)
                        }
                        else -> null
                    }

                    if (targetMapPos != null) {
                        val screenPos = mapToScreen(targetMapPos.first, targetMapPos.second)
                        if (screenPos.x >= -30f && screenPos.x <= canvasW + 30f &&
                            screenPos.y >= -30f && screenPos.y <= canvasH + 30f) {
                            val glyphSize = 7.0f
                            val objPath = Path().apply {
                                moveTo(screenPos.x, screenPos.y - glyphSize)
                                lineTo(screenPos.x + glyphSize, screenPos.y)
                                lineTo(screenPos.x, screenPos.y + glyphSize)
                                lineTo(screenPos.x - glyphSize, screenPos.y)
                                close()
                            }
                            val objColor = Color(0xFFE040FB) // Magenta / Purple
                            drawPath(path = objPath, color = objColor.copy(alpha = if (obj.isCoasting) 0.5f else 0.85f))
                            drawPath(path = objPath, color = Color.White, style = Stroke(width = 1.0f))

                            if (obj.threatScore > 0.5f) {
                                drawCircle(
                                    color = RadarRed.copy(alpha = 0.5f),
                                    radius = glyphSize * 1.8f,
                                    center = screenPos,
                                    style = Stroke(width = 1.5f),
                                )
                            }
                        }
                    }
                }
            }
        }

        // Top-Left: Tactical Map Status Overlay
        Row(
            modifier = Modifier
                .align(Alignment.TopStart)
                .padding(8.dp),
            horizontalArrangement = Arrangement.spacedBy(6.dp),
            verticalAlignment = Alignment.CenterVertically,
        ) {
            Box(
                modifier = Modifier
                    .clip(RoundedCornerShape(4.dp))
                    .background(Color(0xFF0F172A).copy(alpha = 0.85f))
                    .border(1.dp, Color(0xFF1E293B), RoundedCornerShape(4.dp))
                    .padding(horizontal = 8.dp, vertical = 3.dp)
            ) {
                Text(
                    text = "LOCAL MAP 6.0×6.0m (0.25m)",
                    color = RadarCyan,
                    fontSize = 9.sp,
                    fontFamily = FontFamily.Monospace,
                    fontWeight = FontWeight.Bold,
                )
            }

            Box(
                modifier = Modifier
                    .clip(RoundedCornerShape(4.dp))
                    .background(Color(0xFF0F172A).copy(alpha = 0.85f))
                    .border(1.dp, Color(0xFF1E293B), RoundedCornerShape(4.dp))
                    .padding(horizontal = 6.dp, vertical = 3.dp)
            ) {
                Text(
                    text = "KF: ${keyframes.size} | LM: ${landmarks.size}",
                    color = TextMuted,
                    fontSize = 9.sp,
                    fontFamily = FontFamily.Monospace,
                )
            }
        }

        // Top-Right: Map Viewport Action Controls
        Row(
            modifier = Modifier
                .align(Alignment.TopEnd)
                .padding(8.dp),
            horizontalArrangement = Arrangement.spacedBy(6.dp),
            verticalAlignment = Alignment.CenterVertically,
        ) {
            // Viewport Center Toggle
            Box(
                modifier = Modifier
                    .clip(RoundedCornerShape(4.dp))
                    .background(Color(0xFF1E293B).copy(alpha = 0.9f))
                    .border(1.dp, if (centerOnRover) RadarCyan else Color(0xFF334155), RoundedCornerShape(4.dp))
                    .clickable { onToggleCenter() }
                    .padding(horizontal = 8.dp, vertical = 4.dp)
            ) {
                Text(
                    text = if (centerOnRover) "CENTER: ROVER" else "CENTER: ORIGIN",
                    color = if (centerOnRover) TextPrimary else TextMuted,
                    fontSize = 9.sp,
                    fontFamily = FontFamily.Monospace,
                    fontWeight = FontWeight.Bold,
                )
            }

            // Zoom Cycle
            Box(
                modifier = Modifier
                    .clip(RoundedCornerShape(4.dp))
                    .background(Color(0xFF1E293B).copy(alpha = 0.9f))
                    .clickable { onCycleZoom() }
                    .padding(horizontal = 8.dp, vertical = 4.dp)
            ) {
                Text(
                    text = "${String.format(Locale.US, "%.1f", zoom)}x",
                    color = TextPrimary,
                    fontSize = 9.sp,
                    fontFamily = FontFamily.Monospace,
                    fontWeight = FontWeight.Bold,
                )
            }

            // Reset Map & Origin Request Button
            Box(
                modifier = Modifier
                    .clip(RoundedCornerShape(4.dp))
                    .background(Color(0xFF7F1D1D).copy(alpha = 0.8f))
                    .clickable { onResetRequest() }
                    .padding(horizontal = 8.dp, vertical = 4.dp)
            ) {
                Text(
                    text = "RESET MAP",
                    color = Color.White,
                    fontSize = 9.sp,
                    fontFamily = FontFamily.Monospace,
                    fontWeight = FontWeight.Bold,
                )
            }
        }

        // Bottom-Left: Selected Cell Info Bar (on tap)
        if (selectedCellInfo != null) {
            Box(
                modifier = Modifier
                    .align(Alignment.BottomStart)
                    .padding(8.dp)
                    .clip(RoundedCornerShape(4.dp))
                    .background(Color(0xFF0F172A).copy(alpha = 0.92f))
                    .border(1.dp, RadarCyan.copy(alpha = 0.5f), RoundedCornerShape(4.dp))
                    .padding(horizontal = 8.dp, vertical = 4.dp)
            ) {
                Text(
                    text = selectedCellInfo!!,
                    color = TextPrimary,
                    fontSize = 9.sp,
                    fontFamily = FontFamily.Monospace,
                )
            }
        }

        // Bottom-Right: Map Legend Indicator
        Row(
            modifier = Modifier
                .align(Alignment.BottomEnd)
                .padding(8.dp),
            horizontalArrangement = Arrangement.spacedBy(6.dp),
            verticalAlignment = Alignment.CenterVertically,
        ) {
            LegendItem(label = "ROVER", color = MapRoverColor)
            LegendItem(label = "OCC", color = MapOccupiedColor)
            LegendItem(label = "FREE", color = MapFreeColor)
            LegendItem(label = "LM", color = MapLandmarkColor)
            LegendItem(label = "OBJ", color = Color(0xFFE040FB))
        }
    }
}

@Composable
private fun LegendItem(label: String, color: Color) {
    Row(verticalAlignment = Alignment.CenterVertically) {
        Box(
            modifier = Modifier
                .size(6.dp)
                .clip(CircleShape)
                .background(color)
        )
        Spacer(modifier = Modifier.width(3.dp))
        Text(
            text = label,
            color = TextMuted,
            fontSize = 8.sp,
            fontFamily = FontFamily.Monospace,
        )
    }
}
