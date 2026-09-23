package com.minesafety.roboeye.core

import android.app.ActivityManager
import android.content.Context
import androidx.datastore.core.DataStore
import androidx.datastore.preferences.core.Preferences
import androidx.datastore.preferences.core.booleanPreferencesKey
import androidx.datastore.preferences.core.edit
import androidx.datastore.preferences.core.floatPreferencesKey
import androidx.datastore.preferences.core.intPreferencesKey
import androidx.datastore.preferences.core.stringPreferencesKey
import androidx.datastore.preferences.preferencesDataStore
import kotlinx.coroutines.flow.Flow
import kotlinx.coroutines.flow.map

private val Context.dataStore: DataStore<Preferences> by preferencesDataStore(name = "robo_eye_settings")

/** Camera streaming and analysis resolution presets. */
enum class CameraResolution(val label: String, val width: Int, val height: Int) {
  QVGA("320 × 240", 320, 240),
  NHD("640 × 360", 640, 360),
  VGA("640 × 480", 640, 480),
  HD720("1280 × 720", 1280, 720);

  companion object {
    fun fromName(name: String?): CameraResolution =
      entries.firstOrNull { it.name == name } ?: VGA
  }
}

/**
 * Robo Eye configuration.
 */
data class NodeSettings(
  val backendHost: String = DEFAULT_HOST,
  val backendPort: Int = 8000,
  val vehicleId: String = "ROVER-01",
  val username: String = "operator",
  val password: String = "operator123",
  val authToken: String? = null,
  val authRole: String? = null,
  val telemetryHz: Float = 2.0f,
  val cameraFrameHz: Float = 12.0f,
  val cameraResolution: CameraResolution = CameraResolution.VGA,
  val cameraJpegQuality: Int = 70,
  val autoStart: Boolean = false,
  val preferHttpFallback: Boolean = false,
  val pitchOffsetDeg: Float = 0f,
  val rollOffsetDeg: Float = 0f,
  val phonePerceptionEnabled: Boolean = false,
  val radarPublishHz: Float = 2.0f,
  val detectorSensitivity: Float = 0.35f,
  val useCamera2: Boolean = false,
  val visionFps: Float = 10.0f,
  val maxFeatureCount: Int = 150,
  val selectedDetectorModel: com.minesafety.roboeye.perception.model.DetectorModelId =
    com.minesafety.roboeye.perception.model.DetectorModelId.YOLO11N,
) {
  val httpBaseUrl: String get() = "http://$backendHost:$backendPort"

  val wsIngestUrl: String get() = "ws://$backendHost:$backendPort/ws/ingest/$vehicleId"

  val telemetryIntervalMs: Long get() = (1000f / telemetryHz.coerceIn(0.2f, 10f)).toLong()

  val cameraIntervalMs: Long get() = (1000f / cameraFrameHz.coerceIn(1f, 25f)).toLong()

  val hasCredentials: Boolean get() = username.isNotBlank() && password.isNotBlank()

  val isAuthenticated: Boolean get() = !authToken.isNullOrBlank()

  val isHostConfigured: Boolean get() = backendHost.isNotBlank()

  fun toRadarConfig(): RadarConfig =
    RadarConfig(
      backendUrl = httpBaseUrl,
      vehicleId = vehicleId,
      operatorUsername = username,
      operatorPassword = password,
      uplinkRateHz = radarPublishHz,
      detectionThreshold = detectorSensitivity,
    )

  companion object {
    const val DEFAULT_HOST = "10.144.146.199"
  }
}

/**
 * Radar subsystem configuration.
 */
data class RadarConfig(
    val backendUrl: String = DEFAULT_BACKEND_URL,
    val vehicleId: String = "ROVER-01",
    val nodeId: String = DEFAULT_NODE_ID,
    val operatorUsername: String = "operator",
    val operatorPassword: String = "operator123",
    val uplinkRateHz: Float = 2.0f,
    val detectionThreshold: Float = 0.35f,
) {
    val hasCredentials: Boolean
        get() = operatorUsername.isNotBlank() && operatorPassword.isNotBlank()

    val isBackendConfigured: Boolean
        get() = backendUrl.isNotBlank()

    companion object {
        const val DEFAULT_BACKEND_URL = "http://10.144.146.199:8000"
        const val DEFAULT_NODE_ID = "ROBO_EYE_NODE"
    }
}

/**
 * Persisted settings backed by Preferences DataStore.
 */
class SettingsStore(private val context: Context) {

  private val defaultPhonePerception: Boolean by lazy {
    val actManager = context.getSystemService(Context.ACTIVITY_SERVICE) as? ActivityManager
    val memInfo = ActivityManager.MemoryInfo()
    actManager?.getMemoryInfo(memInfo)
    val totalRamGb = memInfo.totalMem / (1024.0 * 1024.0 * 1024.0)
    totalRamGb >= 3.5 // true for Mi 11X (8GB) and S24 Ultra (12GB), false for Redmi 6A (2GB)
  }

  val settings: Flow<NodeSettings> =
    context.dataStore.data.map { p ->
      NodeSettings(
        backendHost = p[KEY_HOST] ?: NodeSettings.DEFAULT_HOST,
        backendPort = p[KEY_PORT] ?: 8000,
        vehicleId = p[KEY_VEHICLE] ?: "ROVER-01",
        username = p[KEY_USER] ?: "operator",
        password = p[KEY_PASS] ?: "operator123",
        authToken = p[KEY_AUTH_TOKEN],
        authRole = p[KEY_AUTH_ROLE],
        telemetryHz = p[KEY_TELEMETRY_HZ] ?: 2.0f,
        cameraFrameHz = p[KEY_CAMERA_HZ] ?: 12.0f,
        cameraResolution = CameraResolution.fromName(p[KEY_CAMERA_RES]),
        cameraJpegQuality = (p[KEY_CAMERA_QUALITY] ?: 70).coerceIn(30, 95),
        autoStart = p[KEY_AUTOSTART] ?: false,
        preferHttpFallback = p[KEY_PREFER_HTTP] ?: false,
        pitchOffsetDeg = p[KEY_PITCH_OFFSET] ?: 0f,
        rollOffsetDeg = p[KEY_ROLL_OFFSET] ?: 0f,
        phonePerceptionEnabled = p[KEY_PHONE_PERCEPTION] ?: defaultPhonePerception,
        radarPublishHz = p[KEY_RADAR_PUBLISH_HZ] ?: 2.0f,
        detectorSensitivity = p[KEY_SENSITIVITY] ?: 0.35f,
        useCamera2 = p[KEY_USE_CAMERA2] ?: false,
        visionFps = p[KEY_VISION_FPS] ?: 10.0f,
        maxFeatureCount = p[KEY_MAX_FEATURES] ?: 150,
        selectedDetectorModel = com.minesafety.roboeye.perception.model.DetectorModelId.fromId(p[KEY_SELECTED_MODEL]),
      )
    }

  suspend fun update(transform: (NodeSettings) -> NodeSettings) {
    context.dataStore.edit { p ->
      val current =
        NodeSettings(
          backendHost = p[KEY_HOST] ?: NodeSettings.DEFAULT_HOST,
          backendPort = p[KEY_PORT] ?: 8000,
          vehicleId = p[KEY_VEHICLE] ?: "ROVER-01",
          username = p[KEY_USER] ?: "operator",
          password = p[KEY_PASS] ?: "operator123",
          authToken = p[KEY_AUTH_TOKEN],
          authRole = p[KEY_AUTH_ROLE],
          telemetryHz = p[KEY_TELEMETRY_HZ] ?: 2.0f,
          cameraFrameHz = p[KEY_CAMERA_HZ] ?: 12.0f,
          cameraResolution = CameraResolution.fromName(p[KEY_CAMERA_RES]),
          cameraJpegQuality = (p[KEY_CAMERA_QUALITY] ?: 70).coerceIn(30, 95),
          autoStart = p[KEY_AUTOSTART] ?: false,
          preferHttpFallback = p[KEY_PREFER_HTTP] ?: false,
          pitchOffsetDeg = p[KEY_PITCH_OFFSET] ?: 0f,
          rollOffsetDeg = p[KEY_ROLL_OFFSET] ?: 0f,
          phonePerceptionEnabled = p[KEY_PHONE_PERCEPTION] ?: defaultPhonePerception,
          radarPublishHz = p[KEY_RADAR_PUBLISH_HZ] ?: 2.0f,
          detectorSensitivity = p[KEY_SENSITIVITY] ?: 0.35f,
          useCamera2 = p[KEY_USE_CAMERA2] ?: false,
          visionFps = p[KEY_VISION_FPS] ?: 10.0f,
          maxFeatureCount = p[KEY_MAX_FEATURES] ?: 150,
          selectedDetectorModel = com.minesafety.roboeye.perception.model.DetectorModelId.fromId(p[KEY_SELECTED_MODEL]),
        )
      val next = transform(current)
      p[KEY_HOST] = next.backendHost.trim()
      p[KEY_PORT] = next.backendPort
      p[KEY_VEHICLE] = next.vehicleId.trim()
      p[KEY_USER] = next.username.trim()
      p[KEY_PASS] = next.password
      if (next.authToken != null) {
        p[KEY_AUTH_TOKEN] = next.authToken
      } else {
        p.remove(KEY_AUTH_TOKEN)
      }
      if (next.authRole != null) {
        p[KEY_AUTH_ROLE] = next.authRole
      } else {
        p.remove(KEY_AUTH_ROLE)
      }
      p[KEY_TELEMETRY_HZ] = next.telemetryHz
      p[KEY_CAMERA_HZ] = next.cameraFrameHz
      p[KEY_CAMERA_RES] = next.cameraResolution.name
      p[KEY_CAMERA_QUALITY] = next.cameraJpegQuality
      p[KEY_AUTOSTART] = next.autoStart
      p[KEY_PREFER_HTTP] = next.preferHttpFallback
      p[KEY_PITCH_OFFSET] = next.pitchOffsetDeg
      p[KEY_ROLL_OFFSET] = next.rollOffsetDeg
      p[KEY_PHONE_PERCEPTION] = next.phonePerceptionEnabled
      p[KEY_RADAR_PUBLISH_HZ] = next.radarPublishHz
      p[KEY_SENSITIVITY] = next.detectorSensitivity
      p[KEY_USE_CAMERA2] = next.useCamera2
      p[KEY_VISION_FPS] = next.visionFps
      p[KEY_MAX_FEATURES] = next.maxFeatureCount
      p[KEY_SELECTED_MODEL] = next.selectedDetectorModel.id
    }
  }

  private companion object {
    val KEY_HOST = stringPreferencesKey("backend_host")
    val KEY_PORT = intPreferencesKey("backend_port")
    val KEY_VEHICLE = stringPreferencesKey("vehicle_id")
    val KEY_USER = stringPreferencesKey("username")
    val KEY_PASS = stringPreferencesKey("password")
    val KEY_AUTH_TOKEN = stringPreferencesKey("auth_token")
    val KEY_AUTH_ROLE = stringPreferencesKey("auth_role")
    val KEY_TELEMETRY_HZ = floatPreferencesKey("telemetry_hz")
    val KEY_CAMERA_HZ = floatPreferencesKey("camera_hz")
    val KEY_CAMERA_RES = stringPreferencesKey("camera_resolution")
    val KEY_CAMERA_QUALITY = intPreferencesKey("camera_jpeg_quality")
    val KEY_AUTOSTART = booleanPreferencesKey("auto_start")
    val KEY_PREFER_HTTP = booleanPreferencesKey("prefer_http_fallback")
    val KEY_PITCH_OFFSET = floatPreferencesKey("pitch_offset_deg")
    val KEY_ROLL_OFFSET = floatPreferencesKey("roll_offset_deg")
    val KEY_PHONE_PERCEPTION = booleanPreferencesKey("phone_perception_enabled")
    val KEY_RADAR_PUBLISH_HZ = floatPreferencesKey("radar_publish_hz")
    val KEY_SENSITIVITY = floatPreferencesKey("detector_sensitivity")
    val KEY_USE_CAMERA2 = booleanPreferencesKey("use_camera2")
    val KEY_VISION_FPS = floatPreferencesKey("vision_fps")
    val KEY_MAX_FEATURES = intPreferencesKey("max_features")
    val KEY_SELECTED_MODEL = stringPreferencesKey("selected_detector_model")
  }
}
