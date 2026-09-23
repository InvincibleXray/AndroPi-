package com.minesafety.roboeye.sensors

import android.Manifest
import android.annotation.SuppressLint
import android.content.Context
import android.content.pm.PackageManager
import android.location.Location
import android.location.LocationManager
import android.os.Looper
import androidx.core.content.ContextCompat
import com.google.android.gms.location.FusedLocationProviderClient
import com.google.android.gms.location.LocationCallback
import com.google.android.gms.location.LocationRequest
import com.google.android.gms.location.LocationResult
import com.google.android.gms.location.LocationServices
import com.google.android.gms.location.Priority
import com.minesafety.roboeye.core.Availability
import com.minesafety.roboeye.core.LocationReading
import com.minesafety.roboeye.core.LocationState
import com.minesafety.roboeye.core.model.SensorStatus
import com.minesafety.roboeye.sensors.location.LocationProvider
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow

/**
 * GPS → `latitude` / `longitude` (+ `speed` when the fix carries it).
 *
 * Underground mines have no GNSS reception, which is a real property of the deployment and
 * not a bug: the node reports `WAITING` / `NOT AVAILABLE` and simply omits the fields.
 * The backend schema already declares both as `float | None`, and nothing in the safety
 * pipeline depends on position.
 *
 * Ground speed comes from the fix rather than being integrated from the accelerometer.
 * Dead-reckoned speed from a phone IMU drifts within seconds, and the backend compares
 * `speed` against real limits (`max_speed_mps`, `fog_speed_limit_mps`), so a drifting
 * estimate would be worse than no value at all. With no GPS speed and no ESP32 encoder,
 * `speed` is omitted.
 */
class LocationSource(private val context: Context) : LocationProvider {

  override val name: String = "Phone Location"

  private val client: FusedLocationProviderClient =
    LocationServices.getFusedLocationProviderClient(context)

  private val _status = MutableStateFlow(SensorStatus.WAITING)
  override val status: StateFlow<SensorStatus> = _status.asStateFlow()

  private val _reading = MutableStateFlow<LocationReading?>(null)
  override val reading: StateFlow<LocationReading?> = _reading.asStateFlow()

  private val _state = MutableStateFlow(LocationState())
  val state: StateFlow<LocationState> = _state.asStateFlow()

  private var running = false

  private val callback =
    object : LocationCallback() {
      override fun onLocationResult(result: LocationResult) {
        val loc: Location = result.lastLocation ?: return
        val r =
          LocationReading(
            latitude = loc.latitude,
            longitude = loc.longitude,
            accuracyM = if (loc.hasAccuracy()) loc.accuracy else null,
            speedMps = if (loc.hasSpeed()) loc.speed else null,
            timestamp = if (loc.time > 0) loc.time else System.currentTimeMillis(),
          )
        _state.value =
          LocationState(
            availability = Availability.AVAILABLE,
            reading = r,
            detail = "",
          )
        _status.value = SensorStatus.ACTIVE
        _reading.value = r
      }
    }

  override fun hasPermission(): Boolean =
    ContextCompat.checkSelfPermission(context, Manifest.permission.ACCESS_FINE_LOCATION) ==
      PackageManager.PERMISSION_GRANTED ||
      ContextCompat.checkSelfPermission(context, Manifest.permission.ACCESS_COARSE_LOCATION) ==
        PackageManager.PERMISSION_GRANTED

  private fun locationEnabled(): Boolean {
    val lm = context.getSystemService(Context.LOCATION_SERVICE) as? LocationManager ?: return false
    return runCatching {
      lm.isProviderEnabled(LocationManager.GPS_PROVIDER) ||
        lm.isProviderEnabled(LocationManager.NETWORK_PROVIDER)
    }
      .getOrDefault(false)
  }

  @SuppressLint("MissingPermission") // guarded by hasPermission() immediately below
  override fun start() {
    if (running) return
    if (!hasPermission()) {
      _state.value =
        LocationState(
          availability = Availability.NO_PERMISSION,
          detail = "Location permission not granted",
        )
      _status.value = SensorStatus.NO_PERMISSION
      return
    }
    if (!locationEnabled()) {
      _state.value =
        LocationState(availability = Availability.UNAVAILABLE, detail = "Location services are off")
      _status.value = SensorStatus.UNAVAILABLE
      return
    }
    running = true
    _state.value = LocationState(availability = Availability.WAITING, detail = "Waiting for fix")
    _status.value = SensorStatus.WAITING
    val request =
      LocationRequest.Builder(Priority.PRIORITY_HIGH_ACCURACY, UPDATE_INTERVAL_MS)
        .setMinUpdateIntervalMillis(MIN_UPDATE_INTERVAL_MS)
        .setWaitForAccurateLocation(false)
        .build()
    runCatching { client.requestLocationUpdates(request, callback, Looper.getMainLooper()) }
      .onFailure { e ->
        running = false
        _state.value =
          LocationState(
            availability = Availability.UNAVAILABLE,
            detail = e.message ?: "Location request failed",
          )
        _status.value = SensorStatus.UNAVAILABLE
      }
  }

  override fun stop() {
    if (!running) return
    running = false
    runCatching { client.removeLocationUpdates(callback) }
    _state.value = LocationState()
    _status.value = SensorStatus.WAITING
    _reading.value = null
  }

  private companion object {
    const val UPDATE_INTERVAL_MS = 1_000L
    const val MIN_UPDATE_INTERVAL_MS = 500L
  }
}
