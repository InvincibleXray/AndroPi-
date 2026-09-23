package com.minesafety.roboeye.ui

import android.app.Application
import androidx.lifecycle.AndroidViewModel
import androidx.lifecycle.viewModelScope
import com.minesafety.roboeye.RoboEyeApp
import com.minesafety.roboeye.core.CameraResolution
import com.minesafety.roboeye.core.NodeSettings
import com.minesafety.roboeye.core.RoboEyeService
import kotlinx.coroutines.delay
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.isActive
import kotlinx.coroutines.launch

enum class RoboEyeTab(val label: String) {
  ROVER_BRAIN("Rover Brain"),
  DIAGNOSTICS("Diagnostics"),
  TELEMETRY("Sensors & Bridge"),
  SETTINGS("Settings"),
  LOGS("Logs"),
}

/**
 * ViewModel powering the unified Rover Brain Android interface.
 */
class RoboEyeViewModel(app: Application) : AndroidViewModel(app) {

  private val controller = RoboEyeApp.controllerOf(app)
  private var autoStartChecked = false

  val status = controller.status
  val settings = controller.settings
  val trackedObjects = controller.trackedObjects
  val worldModel = controller.worldModel
  val semanticObjects = controller.semanticObjects
  val log = controller.logbook.log
  val droppedLogCount = controller.logbook.droppedCount
  val camera = controller.camera

  // Phase 5 Localization & Mapping Runtime Bindings
  val localPose = controller.localizer.localPoseFlow
  val mapState = controller.spatialMapper.mapStateFlow
  val spatialMap = controller.spatialMapper.spatialMap
  val landmarkManager = controller.localizer.landmarkManager

  // Phase 6 Autonomous Navigation Runtime Bindings
  val navigator = controller.navigator
  val navigationState = controller.navigator.stateMachine.snapshot
  val activeGoal = controller.navigator.activeGoal
  val activePath = controller.navigator.activePath
  val lastArbitratedCommand = controller.navigator.lastArbitratedCommand
  val lastSafetyVerdict = controller.navigator.lastSafetyVerdict

  private val _tab = MutableStateFlow(RoboEyeTab.ROVER_BRAIN)
  val tab: StateFlow<RoboEyeTab> = _tab.asStateFlow()

  private val _showBoundingBoxes = MutableStateFlow(true)
  val showBoundingBoxes: StateFlow<Boolean> = _showBoundingBoxes.asStateFlow()

  private val _showGrid = MutableStateFlow(false)
  val showGrid: StateFlow<Boolean> = _showGrid.asStateFlow()

  private val _isTorchEnabled = MutableStateFlow(false)
  val isTorchEnabled: StateFlow<Boolean> = _isTorchEnabled.asStateFlow()

  private val _selectedTargetId = MutableStateFlow<String?>(null)
  val selectedTargetId: StateFlow<String?> = _selectedTargetId.asStateFlow()

  private val _centerOnRover = MutableStateFlow(true)
  val centerOnRover: StateFlow<Boolean> = _centerOnRover.asStateFlow()

  private val _showCameraPip = MutableStateFlow(false)
  val showCameraPip: StateFlow<Boolean> = _showCameraPip.asStateFlow()

  private val _mapZoom = MutableStateFlow(1.0f)
  val mapZoom: StateFlow<Float> = _mapZoom.asStateFlow()

  private val _busy = MutableStateFlow(false)
  val busy: StateFlow<Boolean> = _busy.asStateFlow()

  private val _message = MutableStateFlow<String?>(null)
  val message: StateFlow<String?> = _message.asStateFlow()

  private val _now = MutableStateFlow(System.currentTimeMillis())
  val now: StateFlow<Long> = _now.asStateFlow()

  init {
    viewModelScope.launch {
      while (isActive) {
        _now.value = System.currentTimeMillis()
        delay(1_000)
      }
    }
  }

  fun selectTab(tab: RoboEyeTab) {
    _tab.value = tab
  }

  fun toggleBoundingBoxes() {
    _showBoundingBoxes.value = !_showBoundingBoxes.value
  }

  fun toggleGrid() {
    _showGrid.value = !_showGrid.value
  }

  fun toggleTorch() {
    _isTorchEnabled.value = !_isTorchEnabled.value
  }

  fun selectTarget(id: String?) {
    _selectedTargetId.value = id
  }

  fun consumeMessage() {
    _message.value = null
  }

  fun toggleCenterOnRover() {
    _centerOnRover.value = !_centerOnRover.value
  }

  fun toggleCameraPip() {
    _showCameraPip.value = !_showCameraPip.value
  }

  fun cycleMapZoom() {
    _mapZoom.value = when (_mapZoom.value) {
      1.0f -> 1.5f
      1.5f -> 2.0f
      else -> 1.0f
    }
  }

  fun resetLocalizationAndMap() {
    controller.localizer.resetOrigin()
    controller.spatialMapper.spatialMap.clear()
    controller.navigator.clearGoal()
    _message.value = "Localization origin and local map reset"
  }

  fun setLocalGoal(xM: Float, yM: Float) {
    val goal = com.minesafety.roboeye.nav.model.LocalGoal(xM, yM)
    val wm = com.minesafety.roboeye.nav.model.NavigationWorldModel(
      pose = localPose.value,
      spatialMap = spatialMap,
      mapSummary = mapState.value,
      geometryTrust = controller.status.value.geometricVisionResult?.geometryTrust ?: com.minesafety.roboeye.vision.GeometryTrustLevel.UNTRUSTED,
      motionState = controller.status.value.geometricVisionResult?.motionState ?: com.minesafety.roboeye.vision.MotionState.UNKNOWN,
      recommendedCorridor = controller.status.value.geometricVisionResult?.recommendedCorridor ?: com.minesafety.roboeye.vision.RecommendedCorridor.NONE_AVAILABLE,
      ttcResult = controller.status.value.geometricVisionResult?.ttcResult,
      visualConfidence = controller.status.value.geometricVisionResult?.motionConfidence ?: 0f,
    )
    val res = controller.navigator.setGoal(goal, wm)
    if (res.isValid) {
      _message.value = "Local goal set to (${"%.1f".format(xM)}m, ${"%.1f".format(yM)}m)"
    } else {
      _message.value = "Goal rejected: ${res.reason}"
    }
  }

  fun clearLocalGoal() {
    controller.navigator.clearGoal()
    _message.value = "Local goal cleared"
  }

  fun triggerEmergencyStop() {
    controller.navigator.triggerEmergencyStop("Operator E-Stop button pressed")
    _message.value = "EMERGENCY STOP TRIGGERED"
  }

  fun clearEmergencyStop() {
    controller.navigator.clearEmergencyStop()
    _message.value = "Emergency Stop Cleared"
  }

  fun signIn(host: String, port: Int, vehicleId: String, username: String, password: String) {
    if (_busy.value) return
    viewModelScope.launch {
      _busy.value = true
      controller.applyConnection(host, port, vehicleId)
      controller
        .login(username, password)
        .onSuccess { _message.value = "Signed in as $username" }
        .onFailure { _message.value = "Sign-in failed: ${it.message ?: "unknown error"}" }
      _busy.value = false
    }
  }

  fun signOut() {
    controller.logout()
    _message.value = "Signed out"
  }

  fun startNode() {
    RoboEyeService.start(getApplication())
  }

  fun stopNode() {
    RoboEyeService.stop(getApplication())
  }

  fun calibrateLevel() {
    viewModelScope.launch {
      val ok = controller.calibrateLevel()
      _message.value = if (ok) "Mount level calibrated" else "IMU not available for calibration"
    }
  }

  fun togglePhonePerception(enabled: Boolean) {
    viewModelScope.launch {
      controller.updateSettings { it.copy(phonePerceptionEnabled = enabled) }
      _message.value = if (enabled) "On-Device Neural Perception Enabled" else "Server-Side Perception Active"
    }
  }

  fun toggleUseCamera2(enabled: Boolean) {
    viewModelScope.launch {
      controller.updateSettings { it.copy(useCamera2 = enabled) }
      _message.value = if (enabled) "Camera2 API Requested (Fallback to CameraX on failure)" else "CameraX Engine Active"
    }
  }

  fun selectDetectorModel(modelId: com.minesafety.roboeye.perception.model.DetectorModelId) {
    viewModelScope.launch {
      controller.updateSettings { it.copy(selectedDetectorModel = modelId) }
      _message.value = "Selected detector: ${modelId.displayName}"
    }
  }

  fun startBenchmark(durationSeconds: Long = 60L) {
    val report = controller.startBenchmarkSession(durationMs = durationSeconds * 1000L)
    _message.value = "Benchmark started: ${report.runId} (${report.modelId.displayName})"
  }

  fun stopBenchmark() {
    val report = controller.stopBenchmarkSession()
    _message.value = "Benchmark stopped: ${report.benchmarkStatus.label}"
  }

  fun exportBenchmarkJson(): String? {
    return status.value.benchmarkReport?.toJsonString()
  }

  fun updateSettings(
    host: String,
    port: Int,
    vehicleId: String,
    username: String = settings.value.username,
    password: String = settings.value.password,
    cameraRes: CameraResolution,
    cameraHz: Float,
    telemetryHz: Float,
    phonePerception: Boolean,
    useCamera2: Boolean = settings.value.useCamera2,
    selectedModel: com.minesafety.roboeye.perception.model.DetectorModelId = settings.value.selectedDetectorModel,
  ) {
    viewModelScope.launch {
      controller.updateSettings {
        it.copy(
          backendHost = host.trim(),
          backendPort = port,
          vehicleId = vehicleId.trim(),
          username = username.trim(),
          password = password,
          cameraResolution = cameraRes,
          cameraFrameHz = cameraHz,
          telemetryHz = telemetryHz,
          phonePerceptionEnabled = phonePerception,
          useCamera2 = useCamera2,
          selectedDetectorModel = selectedModel,
        )
      }
      _message.value = "Settings updated"
    }
  }

  fun retryUplink() {
    controller.retryUplink()
  }

  fun onPermissionsChanged() {
    controller.onPermissionsChanged()
  }

  fun maybeAutoStart() {
    if (autoStartChecked) return
    autoStartChecked = true
    viewModelScope.launch {
      val s = controller.loadedSettings()
      if (s.autoStart && s.isHostConfigured) {
        startNode()
      }
    }
  }
}
