package com.minesafety.roboeye.core

import android.content.Context
import androidx.lifecycle.LifecycleOwner
import com.minesafety.roboeye.bridge.BridgeCodec
import com.minesafety.roboeye.bridge.BridgeParse
import com.minesafety.roboeye.bridge.LinkStatus
import com.minesafety.roboeye.bridge.UsbSerialLink
import com.minesafety.roboeye.net.BackendClient
import com.minesafety.roboeye.net.BackendRadarClient
import com.minesafety.roboeye.net.CommandFrame
import com.minesafety.roboeye.net.IngestSocket
import com.minesafety.roboeye.net.VideoTransport
import com.minesafety.roboeye.perception.ByteTracker
import com.minesafety.roboeye.perception.ObjectDetector
import com.minesafety.roboeye.perception.SsdMobileNetDetector
import com.minesafety.roboeye.perception.YoloDetector
import com.minesafety.roboeye.perception.benchmark.BenchmarkMetrics
import com.minesafety.roboeye.perception.benchmark.DeviceBenchmarkReport
import com.minesafety.roboeye.perception.benchmark.DeviceBenchmarkSession
import com.minesafety.roboeye.perception.model.DetectorCreationResult
import com.minesafety.roboeye.perception.model.DetectorFactory
import com.minesafety.roboeye.perception.model.DetectorModelId
import com.minesafety.roboeye.perception.model.ModelReadinessStatus
import com.minesafety.roboeye.perception.model.ModelRegistry
import com.minesafety.roboeye.sensors.BatteryMonitor
import com.minesafety.roboeye.sensors.CameraController
import com.minesafety.roboeye.sensors.DeviceHealth
import com.minesafety.roboeye.sensors.ImuSensor
import com.minesafety.roboeye.sensors.LocationSource
import com.minesafety.roboeye.sensors.NetworkMonitor
import java.util.concurrent.atomic.AtomicBoolean
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.Job
import kotlinx.coroutines.SupervisorJob
import kotlinx.coroutines.delay
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.flow.first
import kotlinx.coroutines.flow.update
import kotlinx.coroutines.isActive
import kotlinx.coroutines.launch
import kotlinx.coroutines.withTimeoutOrNull

/**
 * The unified orchestrator for the Robo Eye Android Node.
 *
 * Combines:
 * 1. Landscape video capture & low-latency streaming to FastAPI backend
 * 2. Mobile sensor telemetry (IMU, Battery, Location, Health) + ESP32 USB-OTG bridge
 * 3. Device-aware on-device AI perception (YOLO11n / SSD MobileNet + ByteTracker)
 * 4. Vision-radar packet uplink to backend
 */
class RoboEyeController(private val appContext: Context) {

  val logbook = Logbook()
  private val scope = CoroutineScope(SupervisorJob() + Dispatchers.Default)

  val settingsStore = SettingsStore(appContext)

  private val _settings = MutableStateFlow(NodeSettings())
  val settings: StateFlow<NodeSettings> = _settings.asStateFlow()

  private val _status = MutableStateFlow(NodeStatus())
  val status: StateFlow<NodeStatus> = _status.asStateFlow()

  private val _trackedObjects = MutableStateFlow<List<TrackedObject>>(emptyList())
  val trackedObjects: StateFlow<List<TrackedObject>> = _trackedObjects.asStateFlow()

  private val _worldModel = MutableStateFlow<com.minesafety.roboeye.nav.model.NavigationWorldModel?>(null)
  val worldModel: StateFlow<com.minesafety.roboeye.nav.model.NavigationWorldModel?> = _worldModel.asStateFlow()

  private val _semanticObjects = MutableStateFlow<List<com.minesafety.roboeye.nav.model.WorldObject>>(emptyList())
  val semanticObjects: StateFlow<List<com.minesafety.roboeye.nav.model.WorldObject>> = _semanticObjects.asStateFlow()

  // Subsystems
  private val imu = ImuSensor(appContext)
  private val location = LocationSource(appContext)
  private val battery = BatteryMonitor(appContext)
  private val network = NetworkMonitor(appContext)
  val health = DeviceHealth(appContext)
  val camera = CameraController(appContext, logbook)
  private val backend = BackendClient(logbook)
  private val socket = IngestSocket(backend.http, scope, logbook)
  val videoTransport = VideoTransport(scope, backend.http, logbook)
  val bridge = UsbSerialLink(appContext, scope, logbook)
  val backendRadarClient = BackendRadarClient()

  // Localization & Spatial Mapping
  val localizer = com.minesafety.roboeye.localization.VisualInertialLocalizer()
  val spatialMapper = com.minesafety.roboeye.mapping.LocalSpatialMapper()

  // Autonomous Navigation, Safety & Motor Transport (Phase 6)
  val usbTransport = com.minesafety.roboeye.esp32.transport.UsbTransport(bridge, scope)
  val safetyController = com.minesafety.roboeye.control.safety.SafetyController(usbTransport)
  val commandSender = com.minesafety.roboeye.nav.transport.RoverCommandSender(usbTransport)
  val navigator = com.minesafety.roboeye.nav.AutonomousNavigator(commandSender = commandSender)

  // Perception & Tracking
  val geometricVisionPipeline = com.minesafety.roboeye.vision.GeometricVisionPipeline(
    localizer = localizer,
    spatialMapper = spatialMapper,
  )
  @Volatile private var latestGeoResult: com.minesafety.roboeye.vision.GeometricVisionResult? = null
  private var detector: ObjectDetector? = null
  private val tracker = ByteTracker()
  val benchmarkMetrics = BenchmarkMetrics()
  val deviceBenchmarkSession = DeviceBenchmarkSession()
  @Volatile private var activeDetectorModelId: DetectorModelId? = null
  @Volatile private var lastInitDurationMs: Long = 0L

  @Volatile private var token: String? = null
  private var uplinkJob: Job? = null
  private var healthJob: Job? = null
  private var navJob: Job? = null
  private val txWindow = ArrayDeque<Long>()
  private var cameraOwner: LifecycleOwner? = null

  init {
    scope.launch { settingsStore.settings.collect { applySettings(it) } }
    scope.launch { imu.state.collect { s -> _status.update { it.copy(imu = s) } } }
    scope.launch { location.state.collect { s -> _status.update { it.copy(location = s) } } }
    scope.launch { battery.state.collect { s -> _status.update { it.copy(battery = s) } } }
    scope.launch { network.state.collect { s -> _status.update { it.copy(network = s) } } }
    scope.launch { health.state.collect { s -> _status.update { it.copy(health = s) } } }
    scope.launch { camera.state.collect { s -> _status.update { it.copy(camera = s) } } }
    scope.launch { socket.state.collect { s -> _status.update { it.copy(backend = it.backend.copy(socket = s)) } } }

    scope.launch {
      bridge.status.collect { linkStatus ->
        _status.update {
          it.copy(
            bridge = it.bridge.copy(
              status = linkStatus,
              deviceName = (linkStatus as? LinkStatus.Connected)?.deviceName ?: it.bridge.deviceName,
            )
          )
        }
      }
    }

    scope.launch {
      bridge.lines.collect { line ->
        val now = System.currentTimeMillis()
        when (val parse = BridgeCodec.parse(line, now)) {
          is BridgeParse.Ok -> {
            _status.update {
              val currentBridge = it.bridge
              it.copy(
                bridge = currentBridge.copy(
                  telemetry = parse.telemetry,
                  lastLineAt = now,
                  linesParsed = currentBridge.linesParsed + 1,
                  recentLines = (listOf(line) + currentBridge.recentLines).take(10),
                )
              )
            }
          }
          is BridgeParse.Rejected -> {
            _status.update {
              val currentBridge = it.bridge
              it.copy(
                bridge = currentBridge.copy(
                  linesRejected = currentBridge.linesRejected + 1,
                  recentLines = (listOf("[REJECT] $line") + currentBridge.recentLines).take(10),
                )
              )
            }
          }
          is BridgeParse.Notice -> {
            _status.update {
              val currentBridge = it.bridge
              it.copy(
                bridge = currentBridge.copy(
                  recentLines = (listOf(line) + currentBridge.recentLines).take(10),
                )
              )
            }
          }
        }
      }
    }

    scope.launch {
      socket.commands.collect { cmd -> onCommand(cmd) }
    }

    scope.launch {
      socket.advisory.collect { adv ->
        _status.update {
          it.copy(
            backend = it.backend.copy(
              advisory = adv,
              advisoryAt = if (adv == null) null else System.currentTimeMillis(),
            )
          )
        }
      }
    }

    videoTransport.onUploaded = { bytes, at, _ ->
      camera.noteUploaded(at)
      _status.update {
        it.copy(
          camera = it.camera.copy(
            streamingFps = videoTransport.currentStreamingFps(),
          )
        )
      }
      if (bytes <= 0) logbook.warn(TAG, "Live video frame accepted with zero bytes")
    }

    videoTransport.onDropped = {
      camera.noteDropped()
      _status.update {
        it.copy(
          camera = it.camera.copy(
            streamingFps = videoTransport.currentStreamingFps(),
          )
        )
      }
    }

    camera.onLiveFrame = { payload, _ ->
      videoTransport.offer(payload)
    }

    geometricVisionPipeline.onGeometricResult = { geoResult ->
      latestGeoResult = geoResult
      _status.update {
        it.copy(
          geometricVisionResult = geoResult,
          localPose = geoResult.localPose,
          localMapState = geoResult.mapState,
          perceptionLatencyMs = geoResult.processingTimeMs,
          perceptionFps = if (geoResult.processingTimeMs > 0) (1000f / geoResult.processingTimeMs).coerceAtMost(30f) else it.perceptionFps,
        )
      }
    }

    camera.onPerceptionFrameSkipped = {
      benchmarkMetrics.recordSkippedFrame()
      deviceBenchmarkSession.onFrameSkipped()
      _status.update { it.copy(benchmarkReport = deviceBenchmarkSession.currentReport(appContext)) }
    }

    camera.onCameraFrame = { frame ->
      val imuReading = _status.value.imu.reading
      geometricVisionPipeline.processFrame(frame, imuReading)
    }

    camera.onPerceptionResult = { result ->
      val s = _settings.value
      val imuReading = _status.value.imu.reading
      benchmarkMetrics.recordSuccess(result.inferenceTimeMs)
      val benchmarkSnap = benchmarkMetrics.snapshot()
      deviceBenchmarkSession.onFrameSubmitted()
      val benchmarkReport = deviceBenchmarkSession.onInferenceSuccess(
        latencyMs = result.inferenceTimeMs,
        detectedObjectsCount = result.detectedObjects.size,
        context = appContext,
      )
      val tracks = tracker.update(
        detections = result.detectedObjects,
        cameraHfovDeg = 68.0f,
        cameraVfovDeg = 52.0f,
        cameraHeightM = 0.95f,
        imuPitchDeg = imuReading?.pitchDeg ?: 0f,
        imuReading = imuReading ?: com.minesafety.roboeye.core.ImuReading(
          isAvailable = _status.value.imu.availability.isUsable,
        ),
      )
      _trackedObjects.value = tracks
      _status.update {
        it.copy(
          radarObjects = tracks,
          phonePerceptionRunning = true,
          perceptionLatencyMs = result.inferenceTimeMs,
          modelName = detector?.modelName ?: "On-Device Neural Detector",
          selectedModelId = s.selectedDetectorModel,
          benchmarkSnapshot = benchmarkSnap,
          benchmarkReport = benchmarkReport,
        )
      }

      if (s.phonePerceptionEnabled && backendRadarClient.connectionStatus.value == com.minesafety.roboeye.core.BackendConnectionStatus.CONNECTED) {
        scope.launch {
          runCatching {
            backendRadarClient.postPerceptionPacket(
              config = s.toRadarConfig(),
              trackedObjects = tracks,
              visibilityScore = result.visibilityScore,
              imuReading = imuReading ?: com.minesafety.roboeye.core.ImuReading(
                isAvailable = _status.value.imu.availability.isUsable,
              ),
              roadGeometry = com.minesafety.roboeye.core.RoadGeometryState(),
            )
          }
        }
      }
    }
  }

  private fun applySettings(s: NodeSettings) {
    _settings.value = s
    val safePitchOffset = if (kotlin.math.abs(s.pitchOffsetDeg) > 40f) 0f else s.pitchOffsetDeg
    val safeRollOffset = if (kotlin.math.abs(s.rollOffsetDeg) > 40f) 0f else s.rollOffsetDeg
    imu.setMountOffsets(safePitchOffset, safeRollOffset)
    _status.update {
      it.copy(
        vehicleId = s.vehicleId,
        backend = it.backend.copy(baseUrl = s.httpBaseUrl),
        selectedModelId = s.selectedDetectorModel,
      )
    }

    camera.phonePerceptionEnabled = s.phonePerceptionEnabled
    camera.sensitivity = s.detectorSensitivity

    val isModelChanged = s.selectedDetectorModel != activeDetectorModelId
    if (s.phonePerceptionEnabled) {
      if (detector == null || isModelChanged) {
        if (isModelChanged && detector != null) {
          camera.phonePerceptionEnabled = false
          tracker.reset()
          benchmarkMetrics.resetForModelSwitch(s.selectedDetectorModel)
          deviceBenchmarkSession.reset()
          _trackedObjects.value = emptyList()
          _semanticObjects.value = emptyList()
        }
        scope.launch(Dispatchers.IO) {
          initDetector(s)
        }
      }
    } else {
      camera.perceptionEngine = null
      camera.phonePerceptionEnabled = false
      activeDetectorModelId = null
      _trackedObjects.value = emptyList()
      _semanticObjects.value = emptyList()
      _worldModel.value = null
      _status.update {
        it.copy(
          radarObjects = emptyList(),
          phonePerceptionRunning = false,
          modelName = "Disabled (Server-Side Inference Active)",
          selectedModelId = s.selectedDetectorModel,
          benchmarkSnapshot = benchmarkMetrics.snapshot(),
        )
      }
    }
  }

  private fun initDetector(s: NodeSettings) {
    try {
      logbook.info(TAG, "Initializing on-device perception detector for model: ${s.selectedDetectorModel.displayName}...")
      val t0 = android.os.SystemClock.elapsedRealtime()
      val creationResult = DetectorFactory.createDetector(s.selectedDetectorModel, appContext)
      val durationMs = android.os.SystemClock.elapsedRealtime() - t0
      lastInitDurationMs = durationMs

      val oldDetector = detector
      detector = creationResult.detector
      camera.perceptionEngine = detector
      activeDetectorModelId = creationResult.activeModelId

      val readiness = when (creationResult) {
        is DetectorCreationResult.Success -> ModelReadinessStatus.READY
        is DetectorCreationResult.Unavailable -> creationResult.status
      }

      benchmarkMetrics.recordInitialization(
        durationMs = durationMs,
        activeModel = creationResult.activeModelId,
        name = creationResult.detector?.modelName ?: "None",
        readiness = readiness,
      )

      if (oldDetector != null && oldDetector !== creationResult.detector) {
        oldDetector.close()
      }

      val isActive = creationResult.detector != null
      camera.phonePerceptionEnabled = isActive
      if (!isActive) {
        _trackedObjects.value = emptyList()
        _semanticObjects.value = emptyList()
      }

      logbook.info(TAG, "Active perception engine: ${detector?.modelName ?: "None (Unavailable)"} (init=${durationMs}ms, message=${creationResult.message})")
      _status.update {
        it.copy(
          selectedModelId = s.selectedDetectorModel,
          phonePerceptionRunning = isActive,
          modelName = detector?.modelName ?: "None (${creationResult.message})",
          benchmarkSnapshot = benchmarkMetrics.snapshot(),
        )
      }
    } catch (e: Exception) {
      logbook.error(TAG, "Failed to initialize detector: ${e.message}")
      detector?.close()
      detector = null
      camera.perceptionEngine = null
      camera.phonePerceptionEnabled = false
      activeDetectorModelId = null
      _trackedObjects.value = emptyList()
      _semanticObjects.value = emptyList()
      benchmarkMetrics.recordInitialization(
        durationMs = 0L,
        activeModel = null,
        name = "None (Error: ${e.message})",
        readiness = ModelReadinessStatus.ERROR,
      )
      _status.update {
        it.copy(
          selectedModelId = s.selectedDetectorModel,
          phonePerceptionRunning = false,
          modelName = "Error: ${e.message}",
          benchmarkSnapshot = benchmarkMetrics.snapshot(),
        )
      }
    }
  }

  suspend fun updateSettings(transform: (NodeSettings) -> NodeSettings) {
    settingsStore.update(transform)
  }

  suspend fun applyConnection(host: String, port: Int, vehicleId: String) {
    val h = host.trim()
    val v = vehicleId.trim()
    settingsStore.update { it.copy(backendHost = h, backendPort = port, vehicleId = v) }
    withTimeoutOrNull(SETTINGS_SETTLE_MS) {
      settings.first { it.backendHost == h && it.backendPort == port && it.vehicleId == v }
    }
  }

  suspend fun loadedSettings(): NodeSettings = settingsStore.settings.first()

  suspend fun calibrateLevel(): Boolean {
    val offsets = imu.calibrateLevel() ?: return false
    settingsStore.update { it.copy(pitchOffsetDeg = offsets.first, rollOffsetDeg = offsets.second) }
    logbook.info(
      TAG,
      "Mount calibrated — pitch offset ${"%.1f".format(offsets.first)}°, " +
        "roll offset ${"%.1f".format(offsets.second)}°",
    )
    return true
  }

  fun startBenchmarkSession(durationMs: Long = 60_000L, warmupFrames: Int = 10): DeviceBenchmarkReport {
    val s = _settings.value
    val report = deviceBenchmarkSession.startSession(
      model = s.selectedDetectorModel,
      activeModel = activeDetectorModelId,
      context = appContext,
      warmupFrames = warmupFrames,
      durationMs = durationMs,
      camResolution = s.cameraResolution.label,
      camTargetFps = 30,
      obsCameraFps = _status.value.camera.analysisFps,
      targetPerceptionHz = camera.maxPerceptionHz,
      backend = detector?.preferredBackend?.label ?: "CPU",
      confThresh = s.detectorSensitivity,
      nmsThresh = 0.45f,
      initDurationMs = lastInitDurationMs,
    )
    _status.update { it.copy(benchmarkReport = report) }
    return report
  }

  fun stopBenchmarkSession(): DeviceBenchmarkReport {
    val report = deviceBenchmarkSession.cancel("USER_STOPPED", appContext)
    _status.update { it.copy(benchmarkReport = report) }
    return report
  }

  suspend fun login(username: String, password: String, persist: Boolean = true): Result<String> {
    val s = _settings.value
    val result = backend.login(s.httpBaseUrl, username, password)
    result
      .onSuccess { resp ->
        token = resp.accessToken
        _status.update {
          it.copy(
            backend =
              it.backend.copy(
                loggedIn = true,
                username = resp.username,
                role = resp.role,
                lastError = null,
              )
          )
        }
        logbook.info(TAG, "Authenticated as ${resp.username} (${resp.role})")

        // Propagate token to subsystems
        videoTransport.updateToken(resp.accessToken)
        backendRadarClient.setCachedToken(resp.accessToken)

        if (_status.value.running) {
          openUplink(s)
        }

        if (persist) {
          settingsStore.update {
            it.copy(
              username = username,
              password = password,
              authToken = resp.accessToken,
              authRole = resp.role,
            )
          }
        }
      }
      .onFailure { e ->
        _status.update {
          it.copy(backend = it.backend.copy(loggedIn = false, lastError = BackendClient.describe(e)))
        }
        logbook.error(TAG, "Login failed: ${BackendClient.describe(e)}")
      }
    return result.map { it.accessToken }
  }

  fun logout() {
    token = null
    videoTransport.updateToken(null)
    backendRadarClient.setCachedToken(null)
    socket.close()
    _status.update {
      it.copy(
        backend = it.backend.copy(
          loggedIn = false,
          username = null,
          role = null,
          socket = com.minesafety.roboeye.core.SocketState.Idle,
          txHz = 0f,
        )
      )
    }
    logbook.info(TAG, "Logged out — switched to standalone mode")
    scope.launch {
      settingsStore.update {
        it.copy(authToken = null, authRole = null, password = "")
      }
    }
  }

  suspend fun start(owner: LifecycleOwner) {
    if (_status.value.running) return
    val s = settingsStore.settings.first()
    _settings.value = s
    cameraOwner = owner
    _status.update { it.copy(running = true, startedAt = System.currentTimeMillis()) }
    logbook.info(TAG, "Robo Eye starting — vehicle ${s.vehicleId}, backend ${s.httpBaseUrl}")

    network.start()
    battery.start()
    val safePitchOffset = if (kotlin.math.abs(s.pitchOffsetDeg) > 40f) 0f else s.pitchOffsetDeg
    val safeRollOffset = if (kotlin.math.abs(s.rollOffsetDeg) > 40f) 0f else s.rollOffsetDeg
    imu.setMountOffsets(safePitchOffset, safeRollOffset)
    imu.start()
    location.start()
    camera.refreshPermission()
    if (camera.hasPermission()) {
      camera.start(owner, s.cameraResolution, s.cameraIntervalMs, s.cameraJpegQuality)
    } else {
      logbook.warn(TAG, "Camera permission missing — streaming will report NOT AVAILABLE")
    }

    bridge.open()

    ensureToken(s)
    probeHealth(s)
    openUplink(s)
    videoTransport.start(s.httpBaseUrl, token, s.vehicleId)

    if (token != null) {
      backendRadarClient.setCachedToken(token)
    } else if (s.phonePerceptionEnabled && s.hasCredentials) {
      scope.launch(Dispatchers.IO) {
        backendRadarClient.authenticate(s.toRadarConfig())
      }
    }

    startUplinkLoop()
    startHealthLoop()
    startNavigationLoop()
  }

  fun stop() {
    if (!_status.value.running) return
    logbook.info(TAG, "Robo Eye stopping")
    navJob?.cancel()
    navJob = null
    uplinkJob?.cancel()
    uplinkJob = null
    healthJob?.cancel()
    healthJob = null
    socket.close()
    videoTransport.stop()
    camera.stop()
    bridge.close()
    imu.stop()
    location.stop()
    battery.stop()
    network.stop()
    txWindow.clear()
    cameraOwner = null
    _trackedObjects.value = emptyList()
    _semanticObjects.value = emptyList()
    _worldModel.value = null
    _status.update {
      it.copy(
        running = false,
        startedAt = null,
        backend = it.backend.copy(socket = com.minesafety.roboeye.core.SocketState.Idle, txHz = 0f),
      )
    }
  }

  fun onPermissionsChanged() {
    camera.refreshPermission()
    if (!_status.value.running) return
    if (location.hasPermission()) location.start()
    val owner = cameraOwner ?: return
    if (camera.hasPermission() && !_status.value.camera.isActive) {
      val s = _settings.value
      scope.launch { camera.start(owner, s.cameraResolution, s.cameraIntervalMs, s.cameraJpegQuality) }
    }
  }

  fun shutdown() {
    stop()
    camera.shutdown()
    detector?.close()
    detector = null
  }

  private suspend fun ensureToken(s: NodeSettings) {
    if (token != null) return
    if (!s.authToken.isNullOrBlank()) {
      token = s.authToken
      _status.update {
        it.copy(
          backend = it.backend.copy(
            loggedIn = true,
            username = s.username,
            role = s.authRole ?: "operator",
            lastError = null,
          )
        )
      }
      videoTransport.updateToken(s.authToken)
      backendRadarClient.setCachedToken(s.authToken)
      logbook.info(TAG, "Restored stored auth token for ${s.username}")
      return
    }
    if (!s.hasCredentials) {
      logbook.info(TAG, "No stored credentials — running in standalone mode")
      return
    }
    login(s.username, s.password, persist = true)
  }

  private fun openUplink(s: NodeSettings) {
    val t = token
    if (t == null) {
      _status.update {
        it.copy(backend = it.backend.copy(socket = com.minesafety.roboeye.core.SocketState.Idle))
      }
      return
    }
    _status.update { it.copy(backend = it.backend.copy(transport = com.minesafety.roboeye.core.UplinkTransport.WEBSOCKET)) }
    socket.connect(s.wsIngestUrl, t, s.vehicleId)
  }

  fun retryUplink() {
    scope.launch {
      val s = _settings.value
      ensureToken(s)
      val t = token
      if (t == null) return@launch
      socket.connect(s.wsIngestUrl, t, s.vehicleId)
    }
  }

  private fun startUplinkLoop() {
    uplinkJob?.cancel()
    uplinkJob =
      scope.launch {
        while (isActive) {
          val s = _settings.value
          val started = System.currentTimeMillis()
          runCatching { tick(s, started) }
            .onFailure { logbook.error(TAG, "Uplink tick failed: ${BackendClient.describe(it)}") }
          val elapsed = System.currentTimeMillis() - started
          delay((s.telemetryIntervalMs - elapsed).coerceAtLeast(MIN_TICK_MS))
        }
      }
  }

  private suspend fun tick(s: NodeSettings, now: Long) {
    val current = _status.value
    val assembled =
      TelemetryAssembler.build(
        vehicleId = s.vehicleId,
        now = now,
        imu = current.imu,
        location = current.location,
        battery = current.battery,
        camera = current.camera,
        bridge = current.bridge,
        peakAccel = imu.consumePeakSample(),
        semanticObjects = if (s.phonePerceptionEnabled && current.camera.isActive) {
          _worldModel.value?.semanticObjects ?: emptyList()
        } else {
          emptyList()
        },
      )

    val verdict =
      SafetyMirror.evaluate(
        pitchDeg = assembled.frame.imuPitch,
        rollDeg = assembled.frame.imuRoll,
        accelMagG = assembled.accelMagG,
        minObstacleM = assembled.minObstacleM,
        bridgeConnected = assembled.bridgeFresh,
        visibility = assembled.frame.visibility,
      )

    _status.update {
      it.copy(
        mirror = verdict,
        imuSource = assembled.imuSource,
        bridgeFresh = assembled.bridgeFresh,
      )
    }

    if (current.backend.transport == com.minesafety.roboeye.core.UplinkTransport.WEBSOCKET && socket.isConnected) {
      val sent = socket.send(assembled.frame)
      if (sent) {
        noteTx(now)
        _status.update {
          it.copy(
            backend =
              it.backend.copy(
                lastSentAt = now,
                framesSent = it.backend.framesSent + 1,
                txHz = currentTxHz(),
                lastError = null,
              )
          )
        }
      } else {
        _status.update {
          it.copy(backend = it.backend.copy(framesFailed = it.backend.framesFailed + 1))
        }
      }
    } else if (current.backend.loggedIn) {
      val t = token ?: return
      val res = backend.postTelemetry(s.httpBaseUrl, t, s.vehicleId, assembled.frame)
      res
        .onSuccess { ack ->
          noteTx(now)
          _status.update {
            it.copy(
              backend =
                it.backend.copy(
                  transport = com.minesafety.roboeye.core.UplinkTransport.HTTP,
                  lastSentAt = now,
                  lastAcceptedAt = now,
                  framesSent = it.backend.framesSent + 1,
                  txHz = currentTxHz(),
                  lastError = null,
                )
            )
          }
          ack.commands.forEach { frame ->
            CommandFrame.from(frame, now)?.let { onCommand(it) }
          }
        }
        .onFailure { e ->
          _status.update {
            it.copy(
              backend =
                it.backend.copy(
                  framesFailed = it.backend.framesFailed + 1,
                  lastError = BackendClient.describe(e),
                )
            )
          }
        }
    }
  }

  private fun startHealthLoop() {
    healthJob?.cancel()
    healthJob =
      scope.launch {
        while (isActive) {
          delay(HEALTH_PROBE_INTERVAL_MS)
          val s = _settings.value
          if (s.isHostConfigured) probeHealth(s)
        }
      }
  }

  private suspend fun probeHealth(s: NodeSettings) {
    val res = backend.health(s.httpBaseUrl)
    res
      .onSuccess { (info, rtt) ->
        _status.update { it.copy(backend = it.backend.copy(latencyMs = rtt, lastError = null)) }
      }
      .onFailure { e ->
        _status.update {
          it.copy(backend = it.backend.copy(latencyMs = null, lastError = BackendClient.describe(e)))
        }
      }
  }

  private fun startNavigationLoop() {
    navJob?.cancel()
    navJob = scope.launch {
      while (isActive) {
        val now = System.currentTimeMillis()
        val geo = latestGeoResult
        val isConnected = bridge.status.value is LinkStatus.Connected

        val currentPose = localizer.localPoseFlow.value
        val semantic = com.minesafety.roboeye.nav.model.ObjectWorldProjection.projectTracks(
          tracks = _trackedObjects.value,
          pose = currentPose,
          nowMs = now,
        )

        val worldModel = com.minesafety.roboeye.nav.model.NavigationWorldModel(
          pose = currentPose,
          spatialMap = spatialMapper.spatialMap,
          mapSummary = spatialMapper.mapStateFlow.value,
          geometryTrust = geo?.geometryTrust ?: com.minesafety.roboeye.vision.GeometryTrustLevel.UNTRUSTED,
          motionState = geo?.motionState ?: com.minesafety.roboeye.vision.MotionState.UNKNOWN,
          recommendedCorridor = geo?.recommendedCorridor ?: com.minesafety.roboeye.vision.RecommendedCorridor.CENTER,
          ttcResult = geo?.ttcResult,
          visualConfidence = geo?.motionConfidence ?: 0f,
          timestampMs = now,
          semanticObjects = semantic,
        )

        _worldModel.value = worldModel
        _semanticObjects.value = semantic

        val safetyState = safetyController.evaluate(
          esp32 = null,
          phone = null,
          visibilityScore = null,
          isTransportConnected = isConnected,
        )

        runCatching {
          navigator.tick(
            worldModel = worldModel,
            safetyState = safetyState,
            isTransportConnected = isConnected,
            nowMs = now,
          )
        }

        delay(100L) // 10 Hz navigation tick
      }
    }
  }

  private fun onCommand(cmd: CommandFrame) {
    logbook.warn(TAG, "Command received: ${cmd.type} (val=${cmd.value}) [seq=${cmd.seq}]")
    _status.update {
      val count = it.bridge.commandsForwarded + 1
      it.copy(
        backend = it.backend.copy(lastCommand = cmd, lastCommandAt = System.currentTimeMillis()),
        bridge = it.bridge.copy(commandsForwarded = count),
      )
    }

    when (cmd.type) {
      "EMERGENCY_STOP" -> navigator.triggerEmergencyStop("Remote emergency stop command received")
      "CLEAR_ESTOP" -> navigator.clearEmergencyStop()
      "STOP" -> navigator.setManualCommand(com.minesafety.roboeye.core.model.MotionCommand.STOP.copy(source = "REMOTE_MANUAL"))
      "MOVE_FORWARD" -> navigator.setManualCommand(com.minesafety.roboeye.core.model.MotionCommand(linearVelocityMps = cmd.value ?: 0.35f, source = "REMOTE_MANUAL"))
      "MOVE_BACKWARD" -> navigator.setManualCommand(com.minesafety.roboeye.core.model.MotionCommand(linearVelocityMps = -(cmd.value ?: 0.35f), source = "REMOTE_MANUAL"))
      "TURN_LEFT" -> navigator.setManualCommand(com.minesafety.roboeye.core.model.MotionCommand(angularVelocityRadS = 0.8f, source = "REMOTE_MANUAL"))
      "TURN_RIGHT" -> navigator.setManualCommand(com.minesafety.roboeye.core.model.MotionCommand(angularVelocityRadS = -0.8f, source = "REMOTE_MANUAL"))
      "SET_SPEED" -> {
        val speed = cmd.value ?: 0.35f
        navigator.setManualCommand(com.minesafety.roboeye.core.model.MotionCommand(linearVelocityMps = speed, source = "REMOTE_MANUAL"))
      }
      else -> {
        // Informational or ping downlink frame
      }
    }
  }

  private fun noteTx(now: Long) {
    txWindow.addLast(now)
    while (txWindow.size > TX_WINDOW_SIZE) txWindow.removeFirst()
  }

  private fun currentTxHz(): Float {
    if (txWindow.size < 2) return 0f
    val span = txWindow.last() - txWindow.first()
    if (span <= 0) return 0f
    return (txWindow.size - 1) * 1000f / span
  }

  companion object {
    private const val TAG = "RoboEyeController"
    private const val SETTINGS_SETTLE_MS = 1_500L
    private const val MIN_TICK_MS = 50L
    private const val HEALTH_PROBE_INTERVAL_MS = 5_000L
    private const val TX_WINDOW_SIZE = 10
  }
}
