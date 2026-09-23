package com.minesafety.roboeye.core

import android.Manifest
import android.app.Notification
import android.app.NotificationChannel
import android.app.NotificationManager
import android.app.PendingIntent
import android.content.Context
import android.content.Intent
import android.content.pm.PackageManager
import android.content.pm.ServiceInfo
import android.os.Build
import android.os.PowerManager
import androidx.core.app.NotificationCompat
import androidx.core.app.ServiceCompat
import androidx.core.content.ContextCompat
import androidx.lifecycle.LifecycleService
import androidx.lifecycle.lifecycleScope
import com.minesafety.roboeye.R
import com.minesafety.roboeye.RoboEyeApp
import com.minesafety.roboeye.ui.MainActivity
import kotlinx.coroutines.flow.collectLatest
import kotlinx.coroutines.launch

/**
 * Foreground service that keeps Robo Eye collecting, streaming video, and running perception
 * continuously with the screen off or on.
 */
class RoboEyeService : LifecycleService() {

  private lateinit var controller: RoboEyeController
  private var wakeLock: PowerManager.WakeLock? = null
  private var nodeStarted = false
  private var lastNotificationText = ""

  override fun onCreate() {
    super.onCreate()
    controller = RoboEyeApp.controllerOf(this)
    createChannel()
  }

  override fun onStartCommand(intent: Intent?, flags: Int, startId: Int): Int {
    super.onStartCommand(intent, flags, startId)

    if (intent?.action == ACTION_STOP) {
      stopNode()
      return START_NOT_STICKY
    }

    if (!promoteToForeground()) {
      stopSelf()
      return START_NOT_STICKY
    }

    if (!nodeStarted) {
      nodeStarted = true
      acquireWakeLock()
      lifecycleScope.launch { controller.start(this@RoboEyeService) }
      observeStatus()
    }
    return START_STICKY
  }

  override fun onDestroy() {
    releaseWakeLock()
    if (nodeStarted) {
      nodeStarted = false
      controller.stop()
    }
    super.onDestroy()
  }

  private fun stopNode() {
    nodeStarted = false
    controller.stop()
    releaseWakeLock()
    ServiceCompat.stopForeground(this, ServiceCompat.STOP_FOREGROUND_REMOVE)
    stopSelf()
  }

  private fun promoteToForeground(): Boolean {
    val notification = buildNotification("Starting Robo Eye…", null)
    return try {
      ServiceCompat.startForeground(this, NOTIFICATION_ID, notification, foregroundTypes())
      true
    } catch (e: Exception) {
      controller.logbook.warn(
        TAG,
        "Foreground start with camera/location types failed (${e.message}); retrying as dataSync",
      )
      try {
        ServiceCompat.startForeground(
          this,
          NOTIFICATION_ID,
          notification,
          ServiceInfo.FOREGROUND_SERVICE_TYPE_DATA_SYNC,
        )
        true
      } catch (e2: Exception) {
        controller.logbook.error(TAG, "Could not start foreground service: ${e2.message}")
        false
      }
    }
  }

  private fun foregroundTypes(): Int {
    var types = ServiceInfo.FOREGROUND_SERVICE_TYPE_DATA_SYNC
    if (granted(Manifest.permission.CAMERA)) {
      types = types or ServiceInfo.FOREGROUND_SERVICE_TYPE_CAMERA
    }
    if (granted(Manifest.permission.ACCESS_FINE_LOCATION) ||
      granted(Manifest.permission.ACCESS_COARSE_LOCATION)
    ) {
      types = types or ServiceInfo.FOREGROUND_SERVICE_TYPE_LOCATION
    }
    return types
  }

  private fun granted(permission: String): Boolean =
    ContextCompat.checkSelfPermission(this, permission) == PackageManager.PERMISSION_GRANTED

  private fun observeStatus() {
    lifecycleScope.launch {
      controller.status.collectLatest { status ->
        val text = summaryOf(status)
        if (text == lastNotificationText) return@collectLatest
        lastNotificationText = text
        val manager = getSystemService(Context.NOTIFICATION_SERVICE) as? NotificationManager
        runCatching {
          manager?.notify(NOTIFICATION_ID, buildNotification(text, warningOf(status)))
        }
      }
    }
  }

  private fun summaryOf(status: NodeStatus): String {
    val link =
      when {
        status.backend.socket is SocketState.Connected -> "LIVE"
        status.backend.socket is SocketState.Connecting -> "connecting"
        status.backend.socket is SocketState.Reconnecting -> "retry ${(status.backend.socket as SocketState.Reconnecting).attempt}"
        !status.backend.loggedIn -> if (status.running) "STANDALONE" else "stopped"
        status.backend.socket is SocketState.RetryExhausted -> "OFFLINE (retries spent)"
        status.backend.socket is SocketState.Failed -> "OFFLINE"
        status.running -> "STANDALONE"
        else -> "stopped"
      }
    val rate = if (status.backend.txHz > 0.05f) " · ${fmt(status.backend.txHz)} Hz" else ""
    val vis =
      status.camera.visibility?.let { " · vis ${it.percent}%" } ?: " · vis NOT AVAILABLE"
    val streamFps = if (status.camera.streamingFps > 0.1f) " · vid ${fmt(status.camera.streamingFps)} fps" else ""
    return "${status.vehicleId} · $link$rate$streamFps$vis"
  }

  private fun warningOf(status: NodeStatus): String? =
    when {
      status.mirror.isEStop -> "LOCAL E-STOP MIRROR: ${status.mirror.reasons.firstOrNull() ?: ""}"
      status.bridge.isSimulated -> "Mock bridge enabled"
      else -> null
    }

  private fun buildNotification(text: String, warning: String?): Notification {
    val contentIntent =
      PendingIntent.getActivity(
        this,
        0,
        Intent(this, MainActivity::class.java).apply {
          flags = Intent.FLAG_ACTIVITY_SINGLE_TOP or Intent.FLAG_ACTIVITY_CLEAR_TOP
        },
        pendingIntentFlags(),
      )
    val stopIntent =
      PendingIntent.getService(
        this,
        1,
        Intent(this, RoboEyeService::class.java).setAction(ACTION_STOP),
        pendingIntentFlags(),
      )
    val builder =
      NotificationCompat.Builder(this, CHANNEL_ID)
        .setSmallIcon(R.drawable.ic_node)
        .setContentTitle("Robo Eye Active")
        .setContentText(text)
        .setContentIntent(contentIntent)
        .addAction(0, "Stop", stopIntent)
        .setOngoing(true)
        .setSilent(true)
        .setCategory(NotificationCompat.CATEGORY_SERVICE)
        .setPriority(NotificationCompat.PRIORITY_LOW)
        .setForegroundServiceBehavior(NotificationCompat.FOREGROUND_SERVICE_IMMEDIATE)
    if (warning != null) {
      builder.setStyle(NotificationCompat.BigTextStyle().bigText("$text\n$warning"))
    }
    return builder.build()
  }

  private fun pendingIntentFlags(): Int =
    if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.M) {
      PendingIntent.FLAG_UPDATE_CURRENT or PendingIntent.FLAG_IMMUTABLE
    } else {
      PendingIntent.FLAG_UPDATE_CURRENT
    }

  private fun createChannel() {
    if (Build.VERSION.SDK_INT < Build.VERSION_CODES.O) return
    val manager = getSystemService(Context.NOTIFICATION_SERVICE) as? NotificationManager ?: return
    val channel =
      NotificationChannel(
          CHANNEL_ID,
          "Robo Eye Background Service",
          NotificationManager.IMPORTANCE_LOW,
        )
        .apply {
          description = "Maintains camera streaming and safety telemetry"
          setShowBadge(false)
        }
    manager.createNotificationChannel(channel)
  }

  private fun acquireWakeLock() {
    if (wakeLock != null) return
    val pm = getSystemService(Context.POWER_SERVICE) as? PowerManager ?: return
    wakeLock =
      runCatching {
          pm.newWakeLock(PowerManager.PARTIAL_WAKE_LOCK, WAKE_LOCK_TAG).apply { acquire() }
        }
        .getOrNull()
  }

  private fun releaseWakeLock() {
    runCatching { wakeLock?.takeIf { it.isHeld }?.release() }
    wakeLock = null
  }

  private fun fmt(v: Float) = String.format(java.util.Locale.US, "%.1f", v)

  companion object {
    private const val TAG = "RoboEyeService"
    private const val CHANNEL_ID = "robo_eye_status"
    private const val NOTIFICATION_ID = 42
    private const val WAKE_LOCK_TAG = "RoboEye:service"

    const val ACTION_START = "com.minesafety.roboeye.action.START"
    const val ACTION_STOP = "com.minesafety.roboeye.action.STOP"

    fun start(context: Context) {
      val intent = Intent(context, RoboEyeService::class.java).setAction(ACTION_START)
      if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
        context.startForegroundService(intent)
      } else {
        context.startService(intent)
      }
    }

    fun stop(context: Context) {
      context.startService(Intent(context, RoboEyeService::class.java).setAction(ACTION_STOP))
    }
  }
}
