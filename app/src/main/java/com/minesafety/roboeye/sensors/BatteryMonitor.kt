package com.minesafety.roboeye.sensors

import android.content.BroadcastReceiver
import android.content.Context
import android.content.Intent
import android.content.IntentFilter
import android.os.BatteryManager
import com.minesafety.roboeye.core.Availability
import com.minesafety.roboeye.core.BatteryState
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow

/**
 * Phone battery → the backend's `battery` field.
 *
 * This is the **handset's** charge, not the rover's traction pack. That distinction is
 * surfaced in the UI ("Phone battery") and in `INTEGRATION.md`, because the backend's
 * `low_battery_pct` / `critical_battery_pct` rules will act on whatever arrives here. If a
 * real pack monitor is later wired to the ESP32, it should override this field in
 * `TelemetryAssembler` rather than be mixed in silently.
 */
class BatteryMonitor(private val context: Context) {

  private val _state = MutableStateFlow(BatteryState())
  val state: StateFlow<BatteryState> = _state.asStateFlow()

  private var registered = false

  private val receiver =
    object : BroadcastReceiver() {
      override fun onReceive(ctx: Context?, intent: Intent?) {
        intent?.let { update(it) }
      }
    }

  fun start() {
    if (registered) return
    val filter = IntentFilter(Intent.ACTION_BATTERY_CHANGED)
    // ACTION_BATTERY_CHANGED is sticky: registering returns the current value immediately.
    val sticky = runCatching { context.registerReceiver(receiver, filter) }.getOrNull()
    registered = true
    if (sticky != null) {
      update(sticky)
    } else {
      _state.value =
        BatteryState(availability = Availability.WAITING, detail = "Waiting for battery broadcast")
    }
  }

  fun stop() {
    if (!registered) return
    registered = false
    runCatching { context.unregisterReceiver(receiver) }
    _state.value = BatteryState()
  }

  private fun update(intent: Intent) {
    val level = intent.getIntExtra(BatteryManager.EXTRA_LEVEL, -1)
    val scale = intent.getIntExtra(BatteryManager.EXTRA_SCALE, -1)
    if (level < 0 || scale <= 0) {
      _state.value =
        BatteryState(availability = Availability.UNAVAILABLE, detail = "Battery level not reported")
      return
    }
    val status = intent.getIntExtra(BatteryManager.EXTRA_STATUS, -1)
    val charging =
      status == BatteryManager.BATTERY_STATUS_CHARGING ||
        status == BatteryManager.BATTERY_STATUS_FULL
    _state.value =
      BatteryState(
        availability = Availability.AVAILABLE,
        percent = (level * 100) / scale,
        charging = charging,
        detail = if (charging) "Charging (USB-OTG hubs with pass-through power)" else "",
      )
  }
}
