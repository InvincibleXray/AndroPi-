package com.minesafety.roboeye.sensors

import android.app.ActivityManager
import android.content.Context
import android.content.Intent
import android.content.IntentFilter
import android.os.BatteryManager
import android.os.Build
import android.os.PowerManager
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow

data class HealthReading(
  val freeRamMb: Long,
  val totalRamMb: Long,
  val ramUsagePercent: Int,
  val batteryTempC: Float?,
  val thermalStatus: String,
  val isLowRamDevice: Boolean,
  val timestamp: Long,
)

data class HealthState(
  val reading: HealthReading? = null,
  val isThrottled: Boolean = false,
)

/**
 * Monitors handset hardware resources: available RAM, thermal status, and battery temperature.
 *
 * Ensures the node detects hardware degradation before an out-of-memory or thermal kill occurs.
 */
class DeviceHealth(private val context: Context) {

  private val _state = MutableStateFlow(HealthState())
  val state: StateFlow<HealthState> = _state.asStateFlow()

  private val activityManager =
    context.getSystemService(Context.ACTIVITY_SERVICE) as? ActivityManager
  private val powerManager =
    context.getSystemService(Context.POWER_SERVICE) as? PowerManager

  fun sample(): HealthReading {
    val memoryInfo = ActivityManager.MemoryInfo()
    activityManager?.getMemoryInfo(memoryInfo)

    val freeRam = memoryInfo.availMem / (1024 * 1024)
    val totalRam = memoryInfo.totalMem / (1024 * 1024)
    val ramPercent =
      if (totalRam > 0) (((totalRam - freeRam).toDouble() / totalRam) * 100).toInt() else 0

    val batteryTemp = readBatteryTemp()
    val thermal = readThermalStatus()
    val throttled = thermal.contains("SEVERE", ignoreCase = true) || thermal.contains("CRITICAL", ignoreCase = true)

    val reading =
      HealthReading(
        freeRamMb = freeRam,
        totalRamMb = totalRam,
        ramUsagePercent = ramPercent,
        batteryTempC = batteryTemp,
        thermalStatus = thermal,
        isLowRamDevice = activityManager?.isLowRamDevice == true,
        timestamp = System.currentTimeMillis(),
      )

    _state.value = HealthState(reading = reading, isThrottled = throttled)
    return reading
  }

  private fun readBatteryTemp(): Float? {
    val intent =
      context.registerReceiver(null, IntentFilter(Intent.ACTION_BATTERY_CHANGED)) ?: return null
    val tempRaw = intent.getIntExtra(BatteryManager.EXTRA_TEMPERATURE, -1)
    return if (tempRaw > 0) tempRaw / 10.0f else null
  }

  private fun readThermalStatus(): String {
    if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.Q && powerManager != null) {
      return when (powerManager.currentThermalStatus) {
        PowerManager.THERMAL_STATUS_NONE -> "NORMAL"
        PowerManager.THERMAL_STATUS_LIGHT -> "LIGHT"
        PowerManager.THERMAL_STATUS_MODERATE -> "MODERATE"
        PowerManager.THERMAL_STATUS_SEVERE -> "SEVERE_THROTTLE"
        PowerManager.THERMAL_STATUS_CRITICAL -> "CRITICAL"
        PowerManager.THERMAL_STATUS_EMERGENCY -> "EMERGENCY"
        PowerManager.THERMAL_STATUS_SHUTDOWN -> "SHUTDOWN"
        else -> "UNKNOWN"
      }
    }
    return "NOT_SUPPORTED"
  }
}
