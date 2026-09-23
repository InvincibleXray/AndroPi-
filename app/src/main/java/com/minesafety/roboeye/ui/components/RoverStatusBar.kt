package com.minesafety.roboeye.ui.components

import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.layout.width
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.foundation.shape.RoundedCornerShape
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
import com.minesafety.roboeye.bridge.LinkStatus
import com.minesafety.roboeye.core.Availability
import com.minesafety.roboeye.core.MirrorLevel
import com.minesafety.roboeye.core.NodeStatus
import com.minesafety.roboeye.core.SocketState
import com.minesafety.roboeye.localization.TrackingQuality
import com.minesafety.roboeye.ui.theme.RadarAmber
import com.minesafety.roboeye.ui.theme.RadarCyan
import com.minesafety.roboeye.ui.theme.RadarGreen
import com.minesafety.roboeye.ui.theme.RadarRed
import com.minesafety.roboeye.ui.theme.TextMuted
import java.util.Locale

/**
 * Compact System Status Bar showing real subsystem health:
 * CAM, IMU, LOC, MAP, LINK, SAFE.
 *
 * Strictly adheres to Rule #3: No fake green indicators.
 * If a subsystem is unavailable or degraded, the real state is rendered honestly.
 */
@Composable
fun RoverStatusBar(
    status: NodeStatus,
    modifier: Modifier = Modifier,
) {
    // 1. Camera Status
    val camActive = status.camera.isActive
    val camFps = status.perceptionFps.takeIf { it > 0 } ?: status.camera.analysisFps
    val (camColor, camLabel) = when {
        camActive && camFps > 0.5f -> RadarGreen to "CAM ${String.format(Locale.US, "%.0f", camFps)}fps"
        camActive -> RadarAmber to "CAM IDLE"
        status.camera.availability == Availability.NO_PERMISSION -> RadarRed to "CAM NO PERM"
        else -> TextMuted to "CAM OFF"
    }

    // 2. IMU Status
    val imuReading = status.imu.reading
    val isImuAvail = status.imu.availability == Availability.AVAILABLE && imuReading != null
    val (imuColor, imuLabel) = when {
        isImuAvail && Math.abs(imuReading.pitchDeg) > 20f -> RadarAmber to "IMU TILT ${imuReading.pitchDeg.toInt()}°"
        isImuAvail -> RadarGreen to "IMU OK"
        status.imu.availability == Availability.WAITING -> RadarAmber to "IMU WAIT"
        else -> TextMuted to "IMU OFF"
    }

    // 3. Localization Status
    val locPose = status.localPose
    val tracking = locPose?.trackingState ?: TrackingQuality.INITIALIZING
    val (locColor, locLabel) = when (tracking) {
        TrackingQuality.TRACKING -> RadarGreen to "LOC TRACK"
        TrackingQuality.DEGRADED -> RadarAmber to "LOC DEG"
        TrackingQuality.LOST -> RadarRed to "LOC LOST"
        TrackingQuality.RELOCALIZING -> RadarCyan to "LOC RELOC"
        TrackingQuality.INITIALIZING -> TextMuted to "LOC INIT"
    }

    // 4. Map Status
    val mapState = status.localMapState
    val (mapColor, mapLabel) = when {
        mapState != null && (mapState.occupiedCellsCount > 0 || mapState.freeCellsCount > 0) ->
            RadarGreen to "MAP ${mapState.occupiedCellsCount}O/${mapState.freeCellsCount}F"
        mapState != null -> RadarCyan to "MAP READY"
        else -> TextMuted to "MAP OFF"
    }

    // 5. Link Status (Backend Uplink or Hardware Bridge)
    val bridgeConnected = status.bridge.status is LinkStatus.Connected
    val backendConnected = status.backend.socket is SocketState.Connected || status.backend.isConnected
    val (linkColor, linkLabel) = when {
        backendConnected -> RadarGreen to "LINK WS"
        bridgeConnected -> RadarCyan to "LINK USB"
        status.backend.socket is SocketState.Connecting -> RadarAmber to "LINK CONN"
        else -> TextMuted to "STANDALONE"
    }

    // 6. Safety Status (Mirror Verdict & Hardware E-Stop)
    val mirror = status.mirror
    val hwEstop = status.bridge.telemetry?.estop == true
    val (safeColor, safeLabel) = when {
        hwEstop || mirror.isEStop || mirror.level == MirrorLevel.LOCAL_ESTOP -> RadarRed to "ESTOP"
        mirror.level == MirrorLevel.CAUTION -> RadarAmber to "CAUTION"
        status.running -> RadarGreen to "SAFE"
        else -> TextMuted to "STANDBY"
    }

    Row(
        modifier = modifier
            .fillMaxWidth()
            .background(Color(0xFF090D16))
            .border(1.dp, Color(0xFF1E293B))
            .padding(horizontal = 8.dp, vertical = 4.dp),
        horizontalArrangement = Arrangement.SpaceBetween,
        verticalAlignment = Alignment.CenterVertically,
    ) {
        StatusIndicator(label = camLabel, color = camColor)
        StatusIndicator(label = imuLabel, color = imuColor)
        StatusIndicator(label = locLabel, color = locColor)
        StatusIndicator(label = mapLabel, color = mapColor)
        StatusIndicator(label = linkLabel, color = linkColor)
        StatusIndicator(label = safeLabel, color = safeColor, isEmphasized = true)
    }
}

@Composable
private fun StatusIndicator(
    label: String,
    color: Color,
    isEmphasized: Boolean = false,
) {
    Row(
        verticalAlignment = Alignment.CenterVertically,
        modifier = if (isEmphasized) {
            Modifier
                .clip(RoundedCornerShape(3.dp))
                .background(color.copy(alpha = 0.2f))
                .padding(horizontal = 6.dp, vertical = 2.dp)
        } else {
            Modifier
        }
    ) {
        Box(
            modifier = Modifier
                .size(6.dp)
                .clip(CircleShape)
                .background(color)
        )
        Spacer(modifier = Modifier.width(4.dp))
        Text(
            text = label,
            color = if (isEmphasized) color else Color(0xFFE2E8F0),
            fontSize = 9.sp,
            fontFamily = FontFamily.Monospace,
            fontWeight = if (isEmphasized) FontWeight.Bold else FontWeight.Normal,
        )
    }
}
