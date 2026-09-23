package com.minesafety.roboeye.ui

import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.clickable
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxHeight
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.layout.width
import androidx.compose.foundation.lazy.LazyColumn
import androidx.compose.foundation.lazy.items
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.foundation.verticalScroll
import androidx.compose.material3.Button
import androidx.compose.material3.ButtonDefaults
import androidx.compose.material3.Card
import androidx.compose.material3.CardDefaults
import androidx.compose.material3.OutlinedTextField
import androidx.compose.material3.OutlinedTextFieldDefaults
import androidx.compose.material3.Scaffold
import androidx.compose.material3.SnackbarHost
import androidx.compose.material3.SnackbarHostState
import androidx.compose.material3.Switch
import androidx.compose.material3.SwitchDefaults
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.collectAsState
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.text.font.FontFamily
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.input.PasswordVisualTransformation
import androidx.compose.material3.AlertDialog
import androidx.compose.ui.platform.LocalClipboardManager
import androidx.compose.ui.text.AnnotatedString
import androidx.compose.ui.text.input.VisualTransformation
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.minesafety.roboeye.bridge.LinkStatus
import com.minesafety.roboeye.core.CameraResolution
import com.minesafety.roboeye.core.MirrorLevel
import com.minesafety.roboeye.core.SocketState
import com.minesafety.roboeye.perception.model.DetectorModelId
import com.minesafety.roboeye.ui.components.AutonomousNavigationCard
import com.minesafety.roboeye.ui.components.CameraPreviewView
import com.minesafety.roboeye.ui.components.FreeSpaceCard
import com.minesafety.roboeye.ui.components.LocalizationCard
import com.minesafety.roboeye.ui.components.LocalMapView
import com.minesafety.roboeye.ui.components.MotionCard
import com.minesafety.roboeye.ui.components.RoverStatusBar
import com.minesafety.roboeye.ui.components.VisualInertialMotionView
import com.minesafety.roboeye.ui.theme.BackgroundDark
import com.minesafety.roboeye.ui.theme.CardBackground
import com.minesafety.roboeye.ui.theme.RadarCyan
import com.minesafety.roboeye.ui.theme.RadarGreen
import com.minesafety.roboeye.ui.theme.RadarRed
import com.minesafety.roboeye.ui.theme.TextMuted
import com.minesafety.roboeye.ui.theme.TextPrimary
import com.minesafety.roboeye.vision.MotionState
import com.minesafety.roboeye.vision.RecommendedCorridor

@Composable
fun RoboEyeScreen(viewModel: RoboEyeViewModel) {
  val status by viewModel.status.collectAsState()
  val settings by viewModel.settings.collectAsState()
  val trackedObjects by viewModel.trackedObjects.collectAsState()
  val activeTab by viewModel.tab.collectAsState()
  val showBoxes by viewModel.showBoundingBoxes.collectAsState()
  val showGrid by viewModel.showGrid.collectAsState()
  val isTorch by viewModel.isTorchEnabled.collectAsState()
  val selectedTargetId by viewModel.selectedTargetId.collectAsState()
  val message by viewModel.message.collectAsState()

  var showResetDialog by remember { mutableStateOf(false) }
  val snackbarHostState = remember { SnackbarHostState() }

  LaunchedEffect(message) {
    message?.let {
      snackbarHostState.showSnackbar(it)
      viewModel.consumeMessage()
    }
  }

  Scaffold(
    snackbarHost = { SnackbarHost(hostState = snackbarHostState) },
    containerColor = BackgroundDark,
  ) { padding ->
    Column(
      modifier = Modifier
        .fillMaxSize()
        .padding(padding)
        .background(BackgroundDark)
    ) {
      // 1. Unified Landscape Top Bar
      TopBar(
        status = status,
        onStart = { viewModel.startNode() },
        onStop = { viewModel.stopNode() },
        onResetRequest = { showResetDialog = true },
      )

      // 2. Navigation Tab Bar
      TabBar(
        activeTab = activeTab,
        onSelectTab = { viewModel.selectTab(it) }
      )

      // 3. Tab Contents
      Box(modifier = Modifier.fillMaxSize()) {
        when (activeTab) {
          RoboEyeTab.ROVER_BRAIN -> {
            RoverBrainView(
              viewModel = viewModel,
              onResetRequest = { showResetDialog = true },
            )
          }
          RoboEyeTab.DIAGNOSTICS -> {
            DiagnosticsView(
              viewModel = viewModel,
              showBoxes = showBoxes,
              showGrid = showGrid,
              isTorch = isTorch,
              selectedTargetId = selectedTargetId,
            )
          }
          RoboEyeTab.TELEMETRY -> {
            TelemetryView(status = status, onCalibrate = { viewModel.calibrateLevel() })
          }
          RoboEyeTab.SETTINGS -> {
            SettingsView(viewModel = viewModel)
          }
          RoboEyeTab.LOGS -> {
            LogsView(viewModel = viewModel)
          }
        }
      }
    }
  }

  // Reset Confirmation Modal
  if (showResetDialog) {
    AlertDialog(
      onDismissRequest = { showResetDialog = false },
      title = {
        Text(
          text = "RESET LOCALIZATION & LOCAL MAP?",
          fontFamily = FontFamily.Monospace,
          fontWeight = FontWeight.Bold,
          fontSize = 13.sp,
          color = RadarRed,
        )
      },
      text = {
        Text(
          text = "This will reset the rover's estimated local pose to (X=0.0m, Y=0.0m, Yaw=0°) and clear all active spatial map cells and visual landmarks. Do you want to proceed?",
          fontSize = 11.sp,
          color = TextPrimary,
        )
      },
      confirmButton = {
        Button(
          onClick = {
            viewModel.resetLocalizationAndMap()
            showResetDialog = false
          },
          colors = ButtonDefaults.buttonColors(containerColor = RadarRed),
          shape = RoundedCornerShape(4.dp),
        ) {
          Text("RESET", fontFamily = FontFamily.Monospace, fontWeight = FontWeight.Bold, fontSize = 11.sp)
        }
      },
      dismissButton = {
        Button(
          onClick = { showResetDialog = false },
          colors = ButtonDefaults.buttonColors(containerColor = Color(0xFF334155)),
          shape = RoundedCornerShape(4.dp),
        ) {
          Text("CANCEL", fontFamily = FontFamily.Monospace, fontSize = 11.sp)
        }
      },
      containerColor = Color(0xFF0F172A),
      shape = RoundedCornerShape(8.dp),
    )
  }
}

@Composable
private fun TopBar(
  status: com.minesafety.roboeye.core.NodeStatus,
  onStart: () -> Unit,
  onStop: () -> Unit,
  onResetRequest: () -> Unit,
) {
  Row(
    modifier = Modifier
      .fillMaxWidth()
      .background(Color(0xFF0F172A))
      .padding(horizontal = 12.dp, vertical = 6.dp),
    verticalAlignment = Alignment.CenterVertically,
    horizontalArrangement = Arrangement.SpaceBetween,
  ) {
    Row(verticalAlignment = Alignment.CenterVertically) {
      Box(
        modifier = Modifier
          .size(10.dp)
          .clip(CircleShape)
          .background(
            when {
              !status.running -> Color(0xFF64748B)
              status.backend.isConnected -> RadarGreen
              status.backend.socket is SocketState.Connecting -> Color(0xFFF59E0B)
              else -> RadarCyan
            }
          )
      )
      Spacer(modifier = Modifier.width(8.dp))
      Text(
        text = "ROVER BRAIN",
        fontFamily = FontFamily.Monospace,
        fontWeight = FontWeight.Bold,
        fontSize = 14.sp,
        color = RadarCyan,
      )
      Spacer(modifier = Modifier.width(12.dp))
      Badge(label = status.vehicleId, color = Color(0xFF334155))

      val (systemStatusLabel, systemStatusColor) = when {
        status.mirror.isEStop || status.bridge.telemetry?.estop == true || status.mirror.level == MirrorLevel.LOCAL_ESTOP -> "E-STOP" to RadarRed
        status.mirror.level == MirrorLevel.CAUTION -> "CAUTION" to Color(0xFFF59E0B)
        status.running -> "ACTIVE" to RadarGreen
        else -> "STANDBY" to Color(0xFF64748B)
      }
      Spacer(modifier = Modifier.width(6.dp))
      Badge(label = systemStatusLabel, color = systemStatusColor.copy(alpha = 0.2f), textColor = systemStatusColor)

      val (socketStatus, socketColor) = when {
        status.backend.socket is SocketState.Connected -> "LIVE" to RadarGreen
        status.backend.socket is SocketState.Connecting -> "CONNECTING" to Color(0xFFF59E0B)
        status.backend.isConnected -> "ONLINE" to RadarGreen
        !status.backend.loggedIn -> "STANDALONE" to Color(0xFF94A3B8)
        else -> "OFFLINE" to TextMuted
      }
      Spacer(modifier = Modifier.width(6.dp))
      Badge(label = socketStatus, color = socketColor.copy(alpha = 0.2f), textColor = socketColor)

      if (status.camera.streamingFps > 0.1f) {
        Spacer(modifier = Modifier.width(6.dp))
        Badge(
          label = "${String.format(java.util.Locale.US, "%.1f", status.camera.streamingFps)} FPS STREAM",
          color = Color(0xFF0E7490).copy(alpha = 0.4f),
          textColor = RadarCyan,
        )
      }
    }

    Row(
      verticalAlignment = Alignment.CenterVertically,
      horizontalArrangement = Arrangement.spacedBy(6.dp)
    ) {
      Button(
        onClick = onResetRequest,
        colors = ButtonDefaults.buttonColors(containerColor = Color(0xFF334155)),
        shape = RoundedCornerShape(6.dp),
      ) {
        Text("RESET", fontFamily = FontFamily.Monospace, fontSize = 11.sp, fontWeight = FontWeight.Bold, color = TextPrimary)
      }

      if (status.running) {
        Button(
          onClick = onStop,
          colors = ButtonDefaults.buttonColors(containerColor = RadarRed),
          shape = RoundedCornerShape(6.dp),
        ) {
          Text("STOP", fontFamily = FontFamily.Monospace, fontSize = 12.sp, fontWeight = FontWeight.Bold)
        }
      } else {
        Button(
          onClick = onStart,
          colors = ButtonDefaults.buttonColors(containerColor = RadarGreen),
          shape = RoundedCornerShape(6.dp),
        ) {
          Text("START", fontFamily = FontFamily.Monospace, fontSize = 12.sp, fontWeight = FontWeight.Bold)
        }
      }
    }
  }
}

@Composable
private fun TabBar(
  activeTab: RoboEyeTab,
  onSelectTab: (RoboEyeTab) -> Unit,
) {
  Row(
    modifier = Modifier
      .fillMaxWidth()
      .background(Color(0xFF090D16))
      .padding(horizontal = 8.dp, vertical = 4.dp),
    horizontalArrangement = Arrangement.spacedBy(8.dp),
  ) {
    RoboEyeTab.entries.forEach { tab ->
      val isSelected = tab == activeTab
      Box(
        modifier = Modifier
          .clip(RoundedCornerShape(6.dp))
          .background(if (isSelected) Color(0xFF1E293B) else Color.Transparent)
          .border(
            1.dp,
            if (isSelected) RadarCyan else Color.Transparent,
            RoundedCornerShape(6.dp)
          )
          .clickable { onSelectTab(tab) }
          .padding(horizontal = 14.dp, vertical = 6.dp)
      ) {
        Text(
          text = tab.label,
          fontFamily = FontFamily.Monospace,
          fontSize = 11.sp,
          fontWeight = if (isSelected) FontWeight.Bold else FontWeight.Normal,
          color = if (isSelected) TextPrimary else TextMuted,
        )
      }
    }
  }
}

@Composable
private fun RoverBrainView(
  viewModel: RoboEyeViewModel,
  onResetRequest: () -> Unit,
) {
  val status by viewModel.status.collectAsState()
  val localPose by viewModel.localPose.collectAsState()
  val mapState by viewModel.mapState.collectAsState()
  val centerOnRover by viewModel.centerOnRover.collectAsState()
  val zoom by viewModel.mapZoom.collectAsState()
  val geoResult = status.geometricVisionResult
  val landmarks = remember(localPose) { viewModel.landmarkManager.activeLandmarks }
  val keyframes = remember(mapState) { viewModel.spatialMap.storedKeyframes }

  val activeGoal by viewModel.activeGoal.collectAsState()
  val activePath by viewModel.activePath.collectAsState()
  val navState by viewModel.navigationState.collectAsState()
  val lastCommand by viewModel.lastArbitratedCommand.collectAsState()
  val semanticObjects by viewModel.semanticObjects.collectAsState()

  Column(
    modifier = Modifier
      .fillMaxSize()
      .background(BackgroundDark)
  ) {
    // 1. Primary Split Screen Area
    Row(
      modifier = Modifier
        .weight(1f)
        .fillMaxWidth()
        .padding(horizontal = 6.dp, vertical = 4.dp),
      horizontalArrangement = Arrangement.spacedBy(6.dp),
    ) {
      // Left: 2D Tactical Local Spatial Map (takes 60% of landscape width)
      Box(
        modifier = Modifier
          .weight(0.60f)
          .fillMaxHeight()
      ) {
        LocalMapView(
          spatialMap = viewModel.spatialMap,
          localPose = localPose,
          landmarks = landmarks,
          keyframes = keyframes,
          activeGoal = activeGoal,
          activePath = activePath,
          semanticObjects = semanticObjects,
          centerOnRover = centerOnRover,
          zoom = zoom,
          onToggleCenter = { viewModel.toggleCenterOnRover() },
          onCycleZoom = { viewModel.cycleMapZoom() },
          onResetRequest = onResetRequest,
          modifier = Modifier.fillMaxSize(),
        )
      }

      // Right: Rover Operator Panels (takes 40% of landscape width)
      Column(
        modifier = Modifier
          .weight(0.40f)
          .fillMaxHeight()
          .verticalScroll(rememberScrollState()),
        verticalArrangement = Arrangement.spacedBy(6.dp),
      ) {
        // Localization Card (True tracking quality, position, scale)
        LocalizationCard(localPose = localPose)

        // Motion Card (Macroscopic motion state, sensor source, confidence)
        MotionCard(
          motionState = geoResult?.motionState ?: MotionState.UNKNOWN,
          motionConfidence = geoResult?.motionConfidence ?: 0.0f,
          isImuAvailable = status.imu.reading != null,
        )

        // Free-Space Card (Phase 4 3-corridor clearance, recommended route, Unknown is NOT safe)
        FreeSpaceCard(
          freeSpace = geoResult?.freeSpace,
          recommendedCorridor = geoResult?.recommendedCorridor ?: RecommendedCorridor.NONE_AVAILABLE,
        )

        // Autonomous Navigation & Safety Control Card (Phase 6)
        AutonomousNavigationCard(
          navigationState = navState,
          activeGoal = activeGoal,
          activePath = activePath,
          lastCommand = lastCommand,
          onSetSampleGoal = {
            val headingRad = Math.toRadians((localPose.yawDeg).toDouble()).toFloat()
            val targetX = localPose.xM + 1.5f * kotlin.math.cos(headingRad)
            val targetY = localPose.yM + 1.5f * kotlin.math.sin(headingRad)
            viewModel.setLocalGoal(targetX, targetY)
          },
          onClearGoal = { viewModel.clearLocalGoal() },
          onEmergencyStop = { viewModel.triggerEmergencyStop() },
          onClearEmergencyStop = { viewModel.clearEmergencyStop() },
        )
      }
    }

    // 2. Compact Bottom Status Bar (CAM, IMU, LOC, MAP, LINK, SAFE)
    RoverStatusBar(status = status)
  }
}

@Composable
private fun DiagnosticsView(
  viewModel: RoboEyeViewModel,
  showBoxes: Boolean,
  showGrid: Boolean,
  isTorch: Boolean,
  selectedTargetId: String?,
) {
  val trackedObjects by viewModel.trackedObjects.collectAsState()
  val semanticObjects by viewModel.semanticObjects.collectAsState()
  val status by viewModel.status.collectAsState()

  Row(
    modifier = Modifier
      .fillMaxSize()
      .padding(6.dp),
    horizontalArrangement = Arrangement.spacedBy(6.dp),
  ) {
    // Left: Live Camera View
    Box(
      modifier = Modifier
        .weight(0.50f)
        .fillMaxHeight()
    ) {
      CameraPreviewView(
        cameraController = viewModel.camera,
        trackedObjects = trackedObjects,
        showBoundingBoxes = showBoxes,
        showGrid = showGrid,
        isTorchEnabled = isTorch,
        isDetectionActive = status.phonePerceptionRunning,
        selectedTargetId = selectedTargetId,
        onToggleBoundingBoxes = { viewModel.toggleBoundingBoxes() },
        onToggleGrid = { viewModel.toggleGrid() },
        onToggleTorch = { viewModel.toggleTorch() },
        onSelectTarget = { viewModel.selectTarget(it) },
        fps = status.camera.streamingFps,
        latencyMs = status.perceptionLatencyMs,
        modifier = Modifier.fillMaxSize(),
      )
    }

    // Right: Real Visual-Inertial Motion Display & Engineering Diagnostics
    Box(
      modifier = Modifier
        .weight(0.50f)
        .fillMaxHeight()
    ) {
      VisualInertialMotionView(
        result = status.geometricVisionResult,
        imuReading = status.imu.reading,
        fps = status.perceptionFps.takeIf { it > 0 } ?: status.camera.analysisFps,
        latencyMs = status.perceptionLatencyMs,
        semanticObjects = semanticObjects,
        modifier = Modifier.fillMaxSize(),
      )
    }
  }
}

@Composable
private fun TelemetryView(
  status: com.minesafety.roboeye.core.NodeStatus,
  onCalibrate: () -> Unit,
) {
  Row(
    modifier = Modifier
      .fillMaxSize()
      .padding(12.dp),
    horizontalArrangement = Arrangement.spacedBy(12.dp),
  ) {
    Column(
      modifier = Modifier
        .weight(1f)
        .verticalScroll(rememberScrollState()),
      verticalArrangement = Arrangement.spacedBy(8.dp),
    ) {
      SectionTitle("PHONE SENSORS")

      InfoCard(title = "Battery") {
        Text("Charge: ${status.battery.percent ?: "—"}%", color = TextPrimary)
        Text("Charging: ${if (status.battery.charging) "YES" else "NO"}", color = TextMuted)
      }

      InfoCard(title = "IMU (Inertial Motion Unit)") {
        val r = status.imu.reading
        Text("Pitch: ${r?.pitchDeg?.let { "%.1f°".format(it) } ?: "—"}", color = TextPrimary)
        Text("Roll: ${r?.rollDeg?.let { "%.1f°".format(it) } ?: "—"}", color = TextPrimary)
        val axStr = r?.let { String.format(java.util.Locale.US, "%.2f", it.axG) } ?: "—"
        val ayStr = r?.let { String.format(java.util.Locale.US, "%.2f", it.ayG) } ?: "—"
        val azStr = r?.let { String.format(java.util.Locale.US, "%.2f", it.azG) } ?: "—"
        Text("Linear Accel: [$axStr, $ayStr, $azStr] g", color = TextMuted)
        Spacer(modifier = Modifier.height(4.dp))
        Button(
          onClick = onCalibrate,
          shape = RoundedCornerShape(4.dp),
          colors = ButtonDefaults.buttonColors(containerColor = Color(0xFF1E293B)),
        ) {
          Text("Calibrate Mount Level", fontSize = 11.sp, fontFamily = FontFamily.Monospace)
        }
      }

      InfoCard(title = "GNSS Location") {
        val loc = status.location.reading
        Text("Lat/Lon: ${loc?.let { "${it.latitude}, ${it.longitude}" } ?: "Searching GPS..."}", color = TextPrimary)
        Text("Ground Speed: ${loc?.speedMps?.let { "%.1f m/s".format(it) } ?: "0.0 m/s"}", color = TextMuted)
      }
    }

    Column(
      modifier = Modifier
        .weight(1f)
        .verticalScroll(rememberScrollState()),
      verticalArrangement = Arrangement.spacedBy(8.dp),
    ) {
      SectionTitle("ESP32 HARDWARE BRIDGE")

      InfoCard(title = "USB-OTG Connection") {
        val linkLabel = status.bridge.status.label
        val connected = status.bridge.status is LinkStatus.Connected
        Text("Status: $linkLabel", color = if (connected) RadarGreen else RadarRed, fontWeight = FontWeight.Bold)
        if (status.bridge.deviceName != null) {
          Text("Device: ${status.bridge.deviceName}", color = TextMuted)
        }
      }

      InfoCard(title = "Chassis Telemetry") {
        val t = status.bridge.telemetry
        Text("Front Ultrasonic: ${t?.ultrasonicFront?.let { "%.2f m".format(it) } ?: "N/A"}", color = TextPrimary)
        Text("Left Ultrasonic: ${t?.ultrasonicLeft?.let { "%.2f m".format(it) } ?: "N/A"}", color = TextPrimary)
        Text("Right Ultrasonic: ${t?.ultrasonicRight?.let { "%.2f m".format(it) } ?: "N/A"}", color = TextPrimary)
        Text("IR Line Sensors: [L:${t?.irLeft ?: "—"}, C:${t?.irCenter ?: "—"}, R:${t?.irRight ?: "—"}]", color = TextMuted)
        Text("Hardware E-Stop: ${t?.estop ?: "OFF"}", color = if (t?.estop == true) RadarRed else RadarGreen)
      }

      SectionTitle("SAFETY MIRROR")
      InfoCard(title = "Local Mirror Verdict") {
        Text("Status: ${status.mirror.level}", color = if (status.mirror.isEStop) RadarRed else RadarGreen, fontWeight = FontWeight.Bold)
        status.mirror.reasons.forEach {
          Text("• $it", color = RadarRed, fontSize = 11.sp)
        }
      }
    }
  }
}

@Composable
private fun SettingsView(viewModel: RoboEyeViewModel) {
  val settings by viewModel.settings.collectAsState()
  val status by viewModel.status.collectAsState()
  val busy by viewModel.busy.collectAsState()

  var host by remember(settings.backendHost) { mutableStateOf(settings.backendHost) }
  var port by remember(settings.backendPort) { mutableStateOf(settings.backendPort.toString()) }
  var vehicleId by remember(settings.vehicleId) { mutableStateOf(settings.vehicleId) }
  var username by remember(settings.username) { mutableStateOf(settings.username) }
  var password by remember(settings.password) { mutableStateOf(settings.password) }
  var passwordVisible by remember { mutableStateOf(false) }
  var phonePerception by remember(settings.phonePerceptionEnabled) { mutableStateOf(settings.phonePerceptionEnabled) }
  var useCamera2 by remember(settings.useCamera2) { mutableStateOf(settings.useCamera2) }
  var selectedModel by remember(settings.selectedDetectorModel) { mutableStateOf(settings.selectedDetectorModel) }
  val clipboardManager = LocalClipboardManager.current
  var copiedNotification by remember { mutableStateOf(false) }

  Column(
    modifier = Modifier
      .fillMaxSize()
      .padding(16.dp)
      .verticalScroll(rememberScrollState()),
    verticalArrangement = Arrangement.spacedBy(12.dp),
  ) {
    SectionTitle("BACKEND CONNECTION")

    Row(horizontalArrangement = Arrangement.spacedBy(12.dp)) {
      OutlinedTextField(
        value = host,
        onValueChange = { host = it },
        label = { Text("Backend Host / IP") },
        modifier = Modifier.weight(2f),
        colors = textFieldColors(),
      )
      OutlinedTextField(
        value = port,
        onValueChange = { port = it },
        label = { Text("Port") },
        modifier = Modifier.weight(1f),
        colors = textFieldColors(),
      )
    }

    OutlinedTextField(
      value = vehicleId,
      onValueChange = { vehicleId = it },
      label = { Text("Vehicle ID") },
      modifier = Modifier.fillMaxWidth(),
      colors = textFieldColors(),
    )

    SectionTitle("BACKEND AUTHENTICATION (OPTIONAL)")

    Card(
      modifier = Modifier.fillMaxWidth(),
      colors = CardDefaults.cardColors(containerColor = CardBackground),
      shape = RoundedCornerShape(8.dp),
      border = androidx.compose.foundation.BorderStroke(1.dp, Color(0xFF1E293B)),
    ) {
      Column(
        modifier = Modifier.padding(12.dp),
        verticalArrangement = Arrangement.spacedBy(8.dp),
      ) {
        Row(
          modifier = Modifier.fillMaxWidth(),
          horizontalArrangement = Arrangement.SpaceBetween,
          verticalAlignment = Alignment.CenterVertically,
        ) {
          Text(
            text = "Status: ${if (status.backend.loggedIn) "AUTHENTICATED" else "STANDALONE MODE"}",
            fontFamily = FontFamily.Monospace,
            fontWeight = FontWeight.Bold,
            fontSize = 12.sp,
            color = if (status.backend.loggedIn) RadarGreen else Color(0xFF94A3B8),
          )
          Badge(
            label = if (status.backend.loggedIn) "UPLINK READY" else "OFFLINE / LOCAL",
            color = (if (status.backend.loggedIn) RadarGreen else Color(0xFF64748B)).copy(alpha = 0.2f),
            textColor = if (status.backend.loggedIn) RadarGreen else Color(0xFF94A3B8),
          )
        }

        if (status.backend.loggedIn) {
          Text(
            text = "Signed in as: ${status.backend.username ?: username} (${status.backend.role ?: "operator"})",
            fontFamily = FontFamily.Monospace,
            fontSize = 12.sp,
            color = TextPrimary,
          )
          Text(
            text = "Live video and telemetry frames are authenticated and streaming to the control room.",
            fontSize = 11.sp,
            color = TextMuted,
          )
          Spacer(modifier = Modifier.height(4.dp))
          Button(
            onClick = { viewModel.signOut() },
            colors = ButtonDefaults.buttonColors(containerColor = Color(0xFF7F1D1D)),
            shape = RoundedCornerShape(6.dp),
            modifier = Modifier.fillMaxWidth(),
          ) {
            Text(
              "SIGN OUT / DISCONNECT UPLINK",
              fontFamily = FontFamily.Monospace,
              fontWeight = FontWeight.Bold,
              color = Color.White,
              fontSize = 12.sp,
            )
          }
        } else {
          Text(
            text = "Authentication is optional. Local camera, AI perception, radar scope, and sensors operate standalone without credentials. Sign in to enable live uplink to FastAPI control room.",
            fontSize = 11.sp,
            color = TextMuted,
          )

          Row(horizontalArrangement = Arrangement.spacedBy(12.dp)) {
            OutlinedTextField(
              value = username,
              onValueChange = { username = it },
              label = { Text("Username") },
              modifier = Modifier.weight(1f),
              colors = textFieldColors(),
              singleLine = true,
            )
            OutlinedTextField(
              value = password,
              onValueChange = { password = it },
              label = { Text("Password") },
              modifier = Modifier.weight(1f),
              colors = textFieldColors(),
              visualTransformation = if (passwordVisible) VisualTransformation.None else PasswordVisualTransformation(),
              trailingIcon = {
                Text(
                  text = if (passwordVisible) "HIDE" else "SHOW",
                  fontFamily = FontFamily.Monospace,
                  fontSize = 10.sp,
                  fontWeight = FontWeight.Bold,
                  color = RadarCyan,
                  modifier = Modifier
                    .clickable { passwordVisible = !passwordVisible }
                    .padding(end = 8.dp),
                )
              },
              singleLine = true,
            )
          }

          if (status.backend.lastError != null) {
            Text(
              text = "Last error: ${status.backend.lastError}",
              fontFamily = FontFamily.Monospace,
              fontSize = 11.sp,
              color = RadarRed,
            )
          }

          Button(
            onClick = {
              val p = port.toIntOrNull() ?: 8000
              viewModel.signIn(host, p, vehicleId, username, password)
            },
            enabled = !busy && username.isNotBlank(),
            colors = ButtonDefaults.buttonColors(containerColor = RadarCyan),
            shape = RoundedCornerShape(6.dp),
            modifier = Modifier.fillMaxWidth(),
          ) {
            Text(
              if (busy) "AUTHENTICATING..." else "SIGN IN & CONNECT UPLINK",
              fontFamily = FontFamily.Monospace,
              fontWeight = FontWeight.Bold,
              color = Color.Black,
              fontSize = 12.sp,
            )
          }
        }
      }
    }

    SectionTitle("PERCEPTION CONFIGURATION")

    Row(
      modifier = Modifier
        .fillMaxWidth()
        .clip(RoundedCornerShape(8.dp))
        .background(CardBackground)
        .padding(12.dp),
      verticalAlignment = Alignment.CenterVertically,
      horizontalArrangement = Arrangement.SpaceBetween,
    ) {
      Column(modifier = Modifier.weight(1f)) {
        Text("On-Device AI Perception (YOLO / TFLite)", fontWeight = FontWeight.Bold, color = TextPrimary)
        Text(
          "Runs real-time neural object detection on handset. Recommended for Mi 11X (8GB) & S24 Ultra (12GB). Disable on low-RAM test phones (Redmi 6A).",
          fontSize = 11.sp,
          color = TextMuted,
        )
      }
      Switch(
        checked = phonePerception,
        onCheckedChange = {
          phonePerception = it
          viewModel.togglePhonePerception(it)
        },
        colors = SwitchDefaults.colors(checkedThumbColor = RadarCyan),
      )
    }

    // Phase 6D: Multi-Model Benchmark & Runtime Selection Card
    Card(
      modifier = Modifier.fillMaxWidth(),
      colors = CardDefaults.cardColors(containerColor = CardBackground),
      shape = RoundedCornerShape(8.dp),
      border = androidx.compose.foundation.BorderStroke(1.dp, Color(0xFF1E293B)),
    ) {
      val yolo26Active = status.phonePerceptionRunning && status.benchmarkSnapshot?.activeModelId == DetectorModelId.YOLO26N

      Column(
        modifier = Modifier.padding(12.dp),
        verticalArrangement = Arrangement.spacedBy(8.dp),
      ) {
        Row(
          modifier = Modifier.fillMaxWidth(),
          horizontalArrangement = Arrangement.SpaceBetween,
          verticalAlignment = Alignment.CenterVertically,
        ) {
          Text(
            text = "Detector Model Selection",
            fontFamily = FontFamily.Monospace,
            fontWeight = FontWeight.Bold,
            fontSize = 12.sp,
            color = RadarCyan,
          )
          Badge(
            label = when {
              selectedModel == DetectorModelId.YOLO26N && yolo26Active -> "READY"
              selectedModel == DetectorModelId.YOLO26N -> "PENDING ASSET / BLOCKED"
              status.phonePerceptionRunning -> "READY"
              else -> "STANDBY"
            },
            color = when {
              selectedModel == DetectorModelId.YOLO26N && !yolo26Active -> Color(0xFF7F1D1D)
              else -> RadarGreen.copy(alpha = 0.2f)
            },
            textColor = when {
              selectedModel == DetectorModelId.YOLO26N && !yolo26Active -> Color(0xFFF87171)
              else -> RadarGreen
            },
          )
        }

        Row(modifier = Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.spacedBy(6.dp)) {
          Button(
            onClick = {
              selectedModel = DetectorModelId.YOLO11N
              viewModel.selectDetectorModel(DetectorModelId.YOLO11N)
            },
            colors = ButtonDefaults.buttonColors(
              containerColor = if (selectedModel == DetectorModelId.YOLO11N) RadarCyan else Color(0xFF1E293B)
            ),
            shape = RoundedCornerShape(4.dp),
            modifier = Modifier.weight(1f),
            contentPadding = androidx.compose.foundation.layout.PaddingValues(horizontal = 4.dp, vertical = 6.dp),
          ) {
            Text(
              "YOLO11n",
              fontFamily = FontFamily.Monospace,
              fontSize = 10.sp,
              fontWeight = FontWeight.Bold,
              color = if (selectedModel == DetectorModelId.YOLO11N) Color.Black else TextPrimary,
            )
          }

          Button(
            onClick = {
              selectedModel = DetectorModelId.YOLO26N
              viewModel.selectDetectorModel(DetectorModelId.YOLO26N)
            },
            colors = ButtonDefaults.buttonColors(
              containerColor = if (selectedModel == DetectorModelId.YOLO26N) Color(0xFFB45309) else Color(0xFF1E293B)
            ),
            shape = RoundedCornerShape(4.dp),
            modifier = Modifier.weight(1f),
            contentPadding = androidx.compose.foundation.layout.PaddingValues(horizontal = 4.dp, vertical = 6.dp),
          ) {
            Text(
              if (yolo26Active) "YOLO26n" else "YOLO26n (Pending)",
              fontFamily = FontFamily.Monospace,
              fontSize = 10.sp,
              fontWeight = FontWeight.Bold,
              color = if (selectedModel == DetectorModelId.YOLO26N) Color.White else TextPrimary,
            )
          }

          Button(
            onClick = {
              selectedModel = DetectorModelId.SSD_MOBILENET
              viewModel.selectDetectorModel(DetectorModelId.SSD_MOBILENET)
            },
            colors = ButtonDefaults.buttonColors(
              containerColor = if (selectedModel == DetectorModelId.SSD_MOBILENET) RadarCyan else Color(0xFF1E293B)
            ),
            shape = RoundedCornerShape(4.dp),
            modifier = Modifier.weight(1f),
            contentPadding = androidx.compose.foundation.layout.PaddingValues(horizontal = 4.dp, vertical = 6.dp),
          ) {
            Text(
              "SSD MobileNet",
              fontFamily = FontFamily.Monospace,
              fontSize = 10.sp,
              fontWeight = FontWeight.Bold,
              color = if (selectedModel == DetectorModelId.SSD_MOBILENET) Color.Black else TextPrimary,
            )
          }
        }

        when (selectedModel) {
          DetectorModelId.YOLO11N -> {
            Text(
              text = "YOLO11n: Standard on-device neural detector. Requires yolov11n_float32.tflite asset.",
              fontFamily = FontFamily.Monospace,
              fontSize = 10.sp,
              color = TextMuted,
            )
          }
          DetectorModelId.YOLO26N -> {
            Text(
              text = if (yolo26Active) {
                "YOLO26n: Active on-device neural detector. Dual head verified, real-time inference operational."
              } else {
                "YOLO26n: Next-generation detector candidate. Model asset is pending delivery; perception is safely paused when selected (no cross-model fallback)."
              },
              fontFamily = FontFamily.Monospace,
              fontSize = 10.sp,
              color = if (yolo26Active) RadarCyan else Color(0xFFFBBF24),
            )
          }
          DetectorModelId.SSD_MOBILENET -> {
            Text(
              text = "SSD MobileNet V1: Baseline benchmark detector. Independent quantized model for comparative profiling.",
              fontFamily = FontFamily.Monospace,
              fontSize = 10.sp,
              color = TextMuted,
            )
          }
        }

        val report = status.benchmarkReport
        val isBenchmarking = report != null && (
          report.benchmarkStatus == com.minesafety.roboeye.perception.benchmark.BenchmarkStatus.WARMING_UP ||
          report.benchmarkStatus == com.minesafety.roboeye.perception.benchmark.BenchmarkStatus.RUNNING_STEADY_STATE
        )

        val isModelReadyForBenchmark = selectedModel != DetectorModelId.YOLO26N || yolo26Active

        Row(modifier = Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.spacedBy(8.dp)) {
          Button(
            onClick = {
              if (isBenchmarking) {
                viewModel.stopBenchmark()
              } else {
                copiedNotification = false
                viewModel.startBenchmark(60L)
              }
            },
            enabled = isModelReadyForBenchmark || isBenchmarking,
            colors = ButtonDefaults.buttonColors(
              containerColor = if (isBenchmarking) Color(0xFF991B1B) else RadarCyan
            ),
            shape = RoundedCornerShape(4.dp),
            modifier = Modifier.weight(1f),
            contentPadding = androidx.compose.foundation.layout.PaddingValues(horizontal = 6.dp, vertical = 6.dp),
          ) {
            Text(
              if (isBenchmarking) "STOP BENCHMARK" else "START 60s BENCHMARK",
              fontFamily = FontFamily.Monospace,
              fontSize = 11.sp,
              fontWeight = FontWeight.Bold,
              color = if (isBenchmarking) Color.White else Color.Black,
            )
          }

          if (report != null) {
            Button(
              onClick = {
                val json = report.toJsonString()
                clipboardManager.setText(AnnotatedString(json))
                copiedNotification = true
              },
              colors = ButtonDefaults.buttonColors(containerColor = Color(0xFF1E293B)),
              shape = RoundedCornerShape(4.dp),
              contentPadding = androidx.compose.foundation.layout.PaddingValues(horizontal = 8.dp, vertical = 6.dp),
            ) {
              Text(
                if (copiedNotification) "COPIED JSON!" else "COPY REPORT",
                fontFamily = FontFamily.Monospace,
                fontSize = 10.sp,
                fontWeight = FontWeight.Bold,
                color = RadarCyan,
              )
            }
          }
        }

        if (report != null) {
          Spacer(modifier = Modifier.height(2.dp))
          Text(
            text = "HARDWARE PROFILE [${report.benchmarkStatus.label.uppercase()}] — ${report.runId}",
            fontFamily = FontFamily.Monospace,
            fontWeight = FontWeight.Bold,
            fontSize = 10.sp,
            color = RadarCyan,
          )
          Text(
            text = "Device: ${report.deviceManufacturer} ${report.deviceModel} (Android ${report.androidVersion}, API ${report.apiLevel}) | Backend: ${report.backend}",
            fontFamily = FontFamily.Monospace,
            fontSize = 10.sp,
            color = TextPrimary,
          )
          Text(
            text = "Warm-up: ${report.warmupCompleted}/${report.warmupTarget} done | Steady Elapsed: ${"%.1f".format(report.elapsedSteadyStateMs / 1000f)}s / 60.0s",
            fontFamily = FontFamily.Monospace,
            fontSize = 10.sp,
            color = TextPrimary,
          )
          Text(
            text = "Steady Latency (${report.inferenceSamples} samples): min ${report.latencyMinMs} ms | p50 ${"%.1f".format(report.latencyP50Ms)} ms | p95 ${"%.1f".format(report.latencyP95Ms)} ms | max ${report.latencyMaxMs} ms",
            fontFamily = FontFamily.Monospace,
            fontSize = 10.sp,
            color = TextPrimary,
          )
          Text(
            text = "Actual Rate: ${"%.2f".format(report.productionInferenceRateHz)} Hz | Max Throughput: ${"%.1f".format(report.theoreticalDetectorThroughputFps)} FPS",
            fontFamily = FontFamily.Monospace,
            fontSize = 10.sp,
            color = TextPrimary,
          )
          Text(
            text = "Frames: ${report.completedFrames} done | ${report.skippedFrames} skipped | ${report.inferenceFailures} failed",
            fontFamily = FontFamily.Monospace,
            fontSize = 10.sp,
            color = TextMuted,
          )
          Text(
            text = "Activity: ${report.totalDetections} detections (${"%.2f".format(report.averageDetectionsPerFrame)}/frame) | Memory: ${"%.1f".format(report.memoryUsageMb)}MB | Thermal: ${report.thermalStatus}",
            fontFamily = FontFamily.Monospace,
            fontSize = 10.sp,
            color = TextMuted,
          )
          if (report.failureReason != null) {
            Text(
              text = "Status Detail: ${report.failureReason}",
              fontFamily = FontFamily.Monospace,
              fontSize = 10.sp,
              color = Color(0xFFF87171),
            )
          }
        } else {
          val snap = status.benchmarkSnapshot
          if (snap != null) {
            Spacer(modifier = Modifier.height(2.dp))
            Text(
              text = "LIVE ROLLING METRICS (${snap.activeModelId?.displayName ?: "NONE / UNAVAILABLE"}):",
              fontFamily = FontFamily.Monospace,
              fontWeight = FontWeight.Bold,
              fontSize = 10.sp,
              color = RadarCyan,
            )
            Text(
              text = "Latency: avg ${"%.1f".format(snap.avgLatencyMs)} ms | p95 ${"%.1f".format(snap.p95LatencyMs)} ms | min ${snap.minLatencyMs} ms | max ${snap.maxLatencyMs} ms",
              fontFamily = FontFamily.Monospace,
              fontSize = 10.sp,
              color = TextPrimary,
            )
            Text(
              text = "Throughput: ${"%.1f".format(snap.detectorThroughputFps)} FPS (max) | Pipeline Rate: ${"%.1f".format(snap.productionRateHz)} Hz",
              fontFamily = FontFamily.Monospace,
              fontSize = 10.sp,
              color = TextPrimary,
            )
            Text(
              text = "Frames: ${snap.successfulInferences} done | ${snap.skippedFrames} skipped | ${snap.failedInferences} failed",
              fontFamily = FontFamily.Monospace,
              fontSize = 10.sp,
              color = TextMuted,
            )
          }
        }
      }
    }

    Row(
      modifier = Modifier
        .fillMaxWidth()
        .clip(RoundedCornerShape(8.dp))
        .background(CardBackground)
        .padding(12.dp),
      verticalAlignment = Alignment.CenterVertically,
      horizontalArrangement = Arrangement.SpaceBetween,
    ) {
      Column(modifier = Modifier.weight(1f)) {
        Text("Use Camera2 API (Experimental)", fontWeight = FontWeight.Bold, color = TextPrimary)
        Text(
          "Direct hardware camera2 access. Default is OFF (CameraX). If unsupported or failed, automatically and silently falls back to CameraX.",
          fontSize = 11.sp,
          color = TextMuted,
        )
      }
      Switch(
        checked = useCamera2,
        onCheckedChange = {
          useCamera2 = it
          viewModel.toggleUseCamera2(it)
        },
        colors = SwitchDefaults.colors(checkedThumbColor = RadarCyan),
      )
    }

    Button(
      onClick = {
        viewModel.updateSettings(
          host = host,
          port = port.toIntOrNull() ?: 8000,
          vehicleId = vehicleId,
          username = username,
          password = password,
          cameraRes = settings.cameraResolution,
          cameraHz = settings.cameraFrameHz,
          telemetryHz = settings.telemetryHz,
          phonePerception = phonePerception,
          useCamera2 = useCamera2,
          selectedModel = selectedModel,
        )
      },
      colors = ButtonDefaults.buttonColors(containerColor = Color(0xFF334155)),
      shape = RoundedCornerShape(6.dp),
      modifier = Modifier.fillMaxWidth(),
    ) {
      Text("SAVE SETTINGS", fontFamily = FontFamily.Monospace, fontWeight = FontWeight.Bold, color = TextPrimary)
    }
  }
}

@Composable
private fun LogsView(viewModel: RoboEyeViewModel) {
  val log by viewModel.log.collectAsState()

  LazyColumn(
    modifier = Modifier
      .fillMaxSize()
      .padding(8.dp)
      .background(Color.Black)
      .border(1.dp, Color(0xFF1E293B), RoundedCornerShape(4.dp))
      .padding(8.dp),
  ) {
    items(log) { entry ->
      val color = when (entry.level) {
        com.minesafety.roboeye.core.LogLevel.ERROR -> RadarRed
        com.minesafety.roboeye.core.LogLevel.WARNING -> Color(0xFFFBBF24)
        else -> TextPrimary
      }
      Text(
        text = "[${entry.tag}] ${entry.message}",
        fontFamily = FontFamily.Monospace,
        fontSize = 10.sp,
        color = color,
      )
    }
  }
}

@Composable
private fun SectionTitle(title: String) {
  Text(
    text = title,
    fontFamily = FontFamily.Monospace,
    fontWeight = FontWeight.Bold,
    fontSize = 12.sp,
    color = RadarCyan,
    modifier = Modifier.padding(vertical = 4.dp),
  )
}

@Composable
private fun InfoCard(title: String, content: @Composable () -> Unit) {
  Card(
    modifier = Modifier.fillMaxWidth(),
    colors = CardDefaults.cardColors(containerColor = CardBackground),
    shape = RoundedCornerShape(8.dp),
    border = androidx.compose.foundation.BorderStroke(1.dp, Color(0xFF1E293B)),
  ) {
    Column(modifier = Modifier.padding(10.dp)) {
      Text(title, fontFamily = FontFamily.Monospace, fontSize = 11.sp, fontWeight = FontWeight.Bold, color = RadarCyan)
      Spacer(modifier = Modifier.height(4.dp))
      content()
    }
  }
}

@Composable
private fun Badge(label: String, color: Color, textColor: Color = TextPrimary) {
  Box(
    modifier = Modifier
      .clip(RoundedCornerShape(4.dp))
      .background(color)
      .padding(horizontal = 6.dp, vertical = 2.dp)
  ) {
    Text(
      text = label,
      fontFamily = FontFamily.Monospace,
      fontSize = 9.sp,
      fontWeight = FontWeight.Bold,
      color = textColor,
    )
  }
}

@Composable
private fun textFieldColors() = OutlinedTextFieldDefaults.colors(
  focusedBorderColor = RadarCyan,
  unfocusedBorderColor = Color(0xFF334155),
  focusedLabelColor = RadarCyan,
  unfocusedLabelColor = TextMuted,
  focusedTextColor = TextPrimary,
  unfocusedTextColor = TextPrimary,
)
