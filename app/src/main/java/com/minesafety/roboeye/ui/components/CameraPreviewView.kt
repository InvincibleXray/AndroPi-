package com.minesafety.roboeye.ui.components

import android.graphics.Paint
import android.graphics.RectF
import android.graphics.Typeface
import android.view.ViewGroup
import androidx.camera.view.PreviewView
import androidx.compose.foundation.Canvas
import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.clickable
import androidx.compose.foundation.gestures.detectTapGestures
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.layout.width
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.FlashlightOff
import androidx.compose.material.icons.filled.FlashlightOn
import androidx.compose.material.icons.filled.GridOff
import androidx.compose.material.icons.filled.GridOn
import androidx.compose.material.icons.filled.Visibility
import androidx.compose.material.icons.filled.VisibilityOff
import androidx.compose.material3.Icon
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.DisposableEffect
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.geometry.Size
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.drawscope.DrawScope
import androidx.compose.ui.graphics.drawscope.Stroke
import androidx.compose.ui.graphics.nativeCanvas
import androidx.compose.ui.graphics.vector.ImageVector
import androidx.compose.ui.input.pointer.pointerInput
import androidx.compose.ui.text.font.FontFamily
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import androidx.compose.ui.viewinterop.AndroidView
import com.minesafety.roboeye.core.PerceptionObjectType
import com.minesafety.roboeye.core.TrackedObject
import com.minesafety.roboeye.sensors.CameraController
import com.minesafety.roboeye.ui.theme.RadarCyan
import com.minesafety.roboeye.ui.theme.RadarRed
import com.minesafety.roboeye.ui.theme.TextMuted
import com.minesafety.roboeye.ui.theme.TextPrimary

/**
 * Camera Preview View with:
 * - Live CameraX Preview feed
 * - Bounding box overlay with non-overlapping collision-resistant labels
 * - HUD reticle / crosshair grid
 * - Floating control buttons (Resolution, Detections, Grid, Torch)
 * - Live telemetry pill (Objects, FPS, Latency)
 * - Interactive tap selection
 */
@Composable
fun CameraPreviewView(
    cameraController: CameraController,
    trackedObjects: List<TrackedObject>,
    showBoundingBoxes: Boolean = true,
    showGrid: Boolean = false,
    isTorchEnabled: Boolean = false,
    isDetectionActive: Boolean = true,
    selectedTargetId: String? = null,
    onToggleDetection: () -> Unit = {},
    onToggleBoundingBoxes: () -> Unit = {},
    onToggleGrid: () -> Unit = {},
    onToggleTorch: () -> Unit = {},
    onSelectTarget: (String?) -> Unit = {},
    fps: Float = 0f,
    latencyMs: Long = 0L,
    isCompact: Boolean = false,
    modifier: Modifier = Modifier,
) {
    DisposableEffect(cameraController) {
        onDispose {
            cameraController.setPreviewSurface(null)
        }
    }

    Box(
        modifier = modifier
            .clip(RoundedCornerShape(10.dp))
            .border(1.dp, Color(0xFF1F2937), RoundedCornerShape(10.dp))
            .background(Color.Black)
    ) {
        // 1. CameraX PreviewView
        AndroidView(
            factory = { ctx ->
                PreviewView(ctx).apply {
                    layoutParams = ViewGroup.LayoutParams(
                        ViewGroup.LayoutParams.MATCH_PARENT,
                        ViewGroup.LayoutParams.MATCH_PARENT
                    )
                    scaleType = PreviewView.ScaleType.FILL_CENTER
                    cameraController.setPreviewSurface(surfaceProvider)
                }
            },
            modifier = Modifier.fillMaxSize()
        )

        // 2. HUD Canvas: Reticle Grid + Bounding Boxes with collision-resistant labels
        Canvas(
            modifier = Modifier
                .fillMaxSize()
                .pointerInput(trackedObjects) {
                    detectTapGestures { tapOffset ->
                        val canvasW = size.width.toFloat()
                        val canvasH = size.height.toFloat()

                        // Check if tap hit any bounding box
                        val hit = trackedObjects.find { obj ->
                            val box = obj.boundingBox
                            val left = box.left * canvasW
                            val top = box.top * canvasH
                            val right = box.right * canvasW
                            val bottom = box.bottom * canvasH
                            tapOffset.x in left..right && tapOffset.y in top..bottom
                        }
                        onSelectTarget(hit?.id)
                    }
                }
        ) {
            val canvasW = size.width
            val canvasH = size.height

            // A. Reticle / Crosshair Overlay (when enabled)
            if (showGrid) {
                drawReticleGrid(canvasW, canvasH)
                drawRoadCorridorOverlay(canvasW, canvasH)
            }

            // B. Bounding Boxes with Collision Avoidance
            if (showBoundingBoxes && isDetectionActive) {
                drawBoundingBoxesWithCollisionAvoidance(
                    trackedObjects = trackedObjects,
                    selectedTargetId = selectedTargetId,
                    canvasW = canvasW,
                    canvasH = canvasH
                )
            }
        }

        // 3. Floating Controls Overlay (hidden in compact PiP mode)
        if (!isCompact) {
            // Floating control pills at Top-Right
            Column(
                modifier = Modifier
                    .align(Alignment.TopEnd)
                    .padding(8.dp),
                verticalArrangement = Arrangement.spacedBy(6.dp),
                horizontalAlignment = Alignment.End
            ) {
                // Resolution Pill
                Box(
                    modifier = Modifier
                        .clip(RoundedCornerShape(4.dp))
                        .background(Color(0xCC0F172A))
                        .border(1.dp, Color(0xFF334155), RoundedCornerShape(4.dp))
                        .padding(horizontal = 6.dp, vertical = 3.dp)
                ) {
                    Text(
                        text = "HD 720p",
                        fontFamily = FontFamily.Monospace,
                        fontSize = 9.sp,
                        fontWeight = FontWeight.Bold,
                        color = RadarCyan
                    )
                }

                // Detections Toggle Pill
                FloatingControlPill(
                    icon = if (showBoundingBoxes) Icons.Default.Visibility else Icons.Default.VisibilityOff,
                    label = "Boxes",
                    isActive = showBoundingBoxes,
                    onClick = onToggleBoundingBoxes
                )

                // Grid Toggle Pill
                FloatingControlPill(
                    icon = if (showGrid) Icons.Default.GridOn else Icons.Default.GridOff,
                    label = "Grid",
                    isActive = showGrid,
                    onClick = onToggleGrid
                )

                // Torch Toggle Pill
                FloatingControlPill(
                    icon = if (isTorchEnabled) Icons.Default.FlashlightOn else Icons.Default.FlashlightOff,
                    label = "Torch",
                    isActive = isTorchEnabled,
                    onClick = onToggleTorch
                )
            }

            // Floating Stats Pill at Bottom-Start
            Box(
                modifier = Modifier
                    .align(Alignment.BottomStart)
                    .padding(8.dp)
                    .clip(RoundedCornerShape(6.dp))
                    .background(Color(0xDD0B0F19))
                    .border(1.dp, Color(0xFF1E293B), RoundedCornerShape(6.dp))
                    .padding(horizontal = 8.dp, vertical = 4.dp)
            ) {
                Text(
                    text = "Objects: ${trackedObjects.size}  |  FPS: ${fps.toInt()}  |  Latency: ${latencyMs}ms",
                    fontFamily = FontFamily.Monospace,
                    fontSize = 10.sp,
                    color = TextPrimary
                )
            }
        }
    }
}

@Composable
private fun FloatingControlPill(
    icon: ImageVector,
    label: String,
    isActive: Boolean,
    onClick: () -> Unit
) {
    Row(
        modifier = Modifier
            .clip(RoundedCornerShape(16.dp))
            .background(if (isActive) Color(0xDD0E7490) else Color(0xAA0F172A))
            .border(1.dp, if (isActive) RadarCyan else Color(0xFF334155), RoundedCornerShape(16.dp))
            .clickable(onClick = onClick)
            .padding(horizontal = 8.dp, vertical = 4.dp),
        verticalAlignment = Alignment.CenterVertically
    ) {
        Icon(
            imageVector = icon,
            contentDescription = label,
            tint = if (isActive) Color.White else TextMuted,
            modifier = Modifier.size(13.dp)
        )
        Spacer(modifier = Modifier.width(4.dp))
        Text(
            text = label,
            fontFamily = FontFamily.Monospace,
            fontSize = 9.sp,
            fontWeight = FontWeight.SemiBold,
            color = if (isActive) Color.White else TextMuted
        )
    }
}

/**
 * Draw Viewfinder Reticle Grid: optical center, horizontal horizon, pitch ticks.
 */
private fun DrawScope.drawReticleGrid(canvasW: Float, canvasH: Float) {
    val cx = canvasW * 0.5f
    val cy = canvasH * 0.5f
    val reticleColor = Color(0x6606B6D4)
    val stroke1 = Stroke(width = 1.dp.toPx())

    // Center crosshair (24dp)
    val crossArm = 14.dp.toPx()
    drawLine(reticleColor, Offset(cx - crossArm, cy), Offset(cx + crossArm, cy), stroke1.width)
    drawLine(reticleColor, Offset(cx, cy - crossArm), Offset(cx, cy + crossArm), stroke1.width)

    // Center circle
    drawCircle(reticleColor, radius = 6.dp.toPx(), style = stroke1)

    // Subtle horizon guides
    drawLine(Color(0x3306B6D4), Offset(canvasW * 0.2f, cy), Offset(canvasW * 0.38f, cy), stroke1.width)
    drawLine(Color(0x3306B6D4), Offset(canvasW * 0.62f, cy), Offset(canvasW * 0.8f, cy), stroke1.width)

    // Pitch ticks (+/- 10%)
    val tickW = 10.dp.toPx()
    val tickY1 = cy - canvasH * 0.15f
    val tickY2 = cy + canvasH * 0.15f
    drawLine(Color(0x3306B6D4), Offset(cx - tickW, tickY1), Offset(cx + tickW, tickY1), stroke1.width)
    drawLine(Color(0x3306B6D4), Offset(cx - tickW, tickY2), Offset(cx + tickW, tickY2), stroke1.width)

    // Viewfinder Corner Brackets
    val bracketLen = 20.dp.toPx()
    val margin = 12.dp.toPx()
    val bracketColor = Color(0x8838BDF8)
    // Top-Left
    drawLine(bracketColor, Offset(margin, margin), Offset(margin + bracketLen, margin), stroke1.width)
    drawLine(bracketColor, Offset(margin, margin), Offset(margin, margin + bracketLen), stroke1.width)
    // Top-Right
    drawLine(bracketColor, Offset(canvasW - margin - bracketLen, margin), Offset(canvasW - margin, margin), stroke1.width)
    drawLine(bracketColor, Offset(canvasW - margin, margin), Offset(canvasW - margin, margin + bracketLen), stroke1.width)
    // Bottom-Left
    drawLine(bracketColor, Offset(margin, canvasH - margin), Offset(margin + bracketLen, canvasH - margin), stroke1.width)
    drawLine(bracketColor, Offset(margin, canvasH - margin), Offset(margin, canvasH - margin - bracketLen), stroke1.width)
    // Bottom-Right
    drawLine(bracketColor, Offset(canvasW - margin - bracketLen, canvasH - margin), Offset(canvasW - margin, canvasH - margin), stroke1.width)
    drawLine(bracketColor, Offset(canvasW - margin, canvasH - margin), Offset(canvasW - margin, canvasH - margin - bracketLen), stroke1.width)
}

private fun DrawScope.drawRoadCorridorOverlay(canvasW: Float, canvasH: Float) {
    val horizonY = canvasH * 0.45f
    val strokeWidth = 1.5.dp.toPx()
    val corridorColor = Color(0x6610B981)

    // Left road boundary line (perspective flaring to bottom edge)
    drawLine(
        color = corridorColor,
        start = Offset(canvasW * 0.42f, horizonY),
        end = Offset(canvasW * 0.08f, canvasH),
        strokeWidth = strokeWidth
    )
    // Right road boundary line
    drawLine(
        color = corridorColor,
        start = Offset(canvasW * 0.58f, horizonY),
        end = Offset(canvasW * 0.92f, canvasH),
        strokeWidth = strokeWidth
    )
}

/**
 * Draw bounding boxes with collision-resistant label positioning algorithm:
 * 1. Detections sorted by distance ascending (closest objects have priority).
 * 2. Attempt placement above box.
 * 3. If overlapping existing labels, attempt placement below box.
 * 4. If still overlapping, abbreviate label.
 * 5. If still overlapping, suppress label text (keep box outline).
 */
private fun DrawScope.drawBoundingBoxesWithCollisionAvoidance(
    trackedObjects: List<TrackedObject>,
    selectedTargetId: String?,
    canvasW: Float,
    canvasH: Float
) {
    val textPaint = Paint().apply {
        textSize = 24f
        isAntiAlias = true
        typeface = Typeface.MONOSPACE
        style = Paint.Style.FILL
    }

    val pillPaint = Paint().apply {
        style = Paint.Style.FILL
        isAntiAlias = true
    }

    // Sorted by distance ascending: closest targets prioritized for label placement
    val sortedObjects = trackedObjects.sortedBy { it.estimatedDistanceM }
    val placedLabelRects = mutableListOf<RectF>()

    for (obj in sortedObjects) {
        val box = obj.boundingBox
        val left = (box.left * canvasW).coerceAtLeast(0f)
        val top = (box.top * canvasH).coerceAtLeast(0f)
        val right = (box.right * canvasW).coerceAtMost(canvasW)
        val bottom = (box.bottom * canvasH).coerceAtMost(canvasH)

        val isSelected = obj.id == selectedTargetId

        val categoryColor = when (obj.type) {
            PerceptionObjectType.VEHICLE -> RadarCyan
            PerceptionObjectType.PERSON -> Color(0xFFFBBF24)
            PerceptionObjectType.OBSTACLE,
            PerceptionObjectType.ROAD_OBSTRUCTION,
            PerceptionObjectType.LARGE_OBJECT -> RadarRed
        }

        val boxColor = if (obj.estimatedDistanceM < 5.0f) RadarRed else categoryColor

        // Draw Bounding Box Rectangle
        val boxStrokeWidth = if (isSelected) 3.5.dp.toPx() else 2.dp.toPx()
        drawRect(
            color = boxColor,
            topLeft = Offset(left, top),
            size = Size((right - left).coerceAtLeast(10f), (bottom - top).coerceAtLeast(10f)),
            style = Stroke(width = boxStrokeWidth)
        )

        // If selected, draw corner accent accents
        if (isSelected) {
            drawCircle(
                color = RadarCyan,
                radius = 4.dp.toPx(),
                center = Offset((left + right) * 0.5f, (top + bottom) * 0.5f)
            )
        }

        // --- Label Collision Resolution ---
        val metricTag = if (obj.metricWidthM > 0.0f) "${obj.metricWidthM}m·" else ""
        val fullText = "${obj.id}: ${obj.label.replaceFirstChar { it.uppercase() }} ($metricTag${obj.estimatedDistanceM}m)"
        val shortText = "${obj.id} ($metricTag${obj.estimatedDistanceM}m)"
        val minText = obj.id

        val labelPaddingX = 8f
        val labelPaddingY = 4f
        val textHeight = 22f

        // Helper to check rect overlap
        fun rectsOverlap(r1: RectF, r2: RectF): Boolean {
            return r1.left < r2.right && r1.right > r2.left && r1.top < r2.bottom && r1.bottom > r2.top
        }

        fun tryPlaceLabel(text: String, candidateY: Float): RectF? {
            val textWidth = textPaint.measureText(text)
            val rect = RectF(
                left,
                candidateY,
                (left + textWidth + labelPaddingX * 2).coerceAtMost(canvasW),
                candidateY + textHeight + labelPaddingY * 2
            )
            val collides = placedLabelRects.any { existing -> rectsOverlap(rect, existing) }
            return if (!collides) rect else null
        }

        // Position candidates
        val candidateAboveY = top - textHeight - labelPaddingY * 2 - 4f
        val candidateBelowY = bottom + 4f

        var chosenText: String? = null
        var chosenRect: RectF? = null

        // Try 1: Full text above
        if (candidateAboveY >= 0f) {
            val r = tryPlaceLabel(fullText, candidateAboveY)
            if (r != null) {
                chosenText = fullText
                chosenRect = r
            }
        }

        // Try 2: Full text below (if above failed or was offscreen)
        if (chosenRect == null && candidateBelowY + textHeight + labelPaddingY * 2 <= canvasH) {
            val r = tryPlaceLabel(fullText, candidateBelowY)
            if (r != null) {
                chosenText = fullText
                chosenRect = r
            }
        }

        // Try 3: Abbreviated text above
        if (chosenRect == null && candidateAboveY >= 0f) {
            val r = tryPlaceLabel(shortText, candidateAboveY)
            if (r != null) {
                chosenText = shortText
                chosenRect = r
            }
        }

        // Try 4: Abbreviated text below
        if (chosenRect == null && candidateBelowY + textHeight + labelPaddingY * 2 <= canvasH) {
            val r = tryPlaceLabel(shortText, candidateBelowY)
            if (r != null) {
                chosenText = shortText
                chosenRect = r
            }
        }

        // Try 5: Minimal ID text
        if (chosenRect == null && candidateAboveY >= 0f) {
            val r = tryPlaceLabel(minText, candidateAboveY)
            if (r != null) {
                chosenText = minText
                chosenRect = r
            }
        }

        // Draw chosen label (or suppress if all candidates collided)
        if (chosenText != null && chosenRect != null) {
            placedLabelRects.add(chosenRect)

            // Draw semi-transparent dark pill background behind label
            pillPaint.color = android.graphics.Color.argb(200, 11, 15, 25)
            drawContext.canvas.nativeCanvas.drawRoundRect(
                chosenRect,
                6f,
                6f,
                pillPaint
            )

            // Draw pill border with category tint
            val borderPaint = Paint().apply {
                style = Paint.Style.STROKE
                this.strokeWidth = 1.5f
                color = boxColor.hashCode()
                isAntiAlias = true
            }
            drawContext.canvas.nativeCanvas.drawRoundRect(
                chosenRect,
                6f,
                6f,
                borderPaint
            )

            // Draw label text
            textPaint.color = android.graphics.Color.WHITE
            drawContext.canvas.nativeCanvas.drawText(
                chosenText,
                chosenRect.left + labelPaddingX,
                chosenRect.bottom - labelPaddingY - 4f,
                textPaint
            )
        }
    }
}
