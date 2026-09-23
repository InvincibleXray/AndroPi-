package com.minesafety.roboeye.sensors

import android.content.Context
import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import com.minesafety.roboeye.core.Availability
import com.minesafety.roboeye.core.ImuReading
import com.minesafety.roboeye.core.ImuState
import com.minesafety.roboeye.core.Units
import com.minesafety.roboeye.core.model.SensorStatus
import com.minesafety.roboeye.sensors.imu.ImuProvider
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlin.math.abs
import kotlin.math.sqrt

/**
 * Phone IMU → the backend's `imu_*` fields.
 *
 * Two sensors are used for two different jobs, which is not interchangeable:
 *
 * * **`TYPE_ACCELEROMETER`** (includes gravity) → `imu_pitch` / `imu_roll`. Orientation is
 *   *derived from* the gravity vector, so a gravity-free signal cannot produce it.
 * * **`TYPE_LINEAR_ACCELERATION`** (gravity removed) → `imu_ax` / `imu_ay` / `imu_az` in g.
 *   The backend's only consumer of these is `sqrt(ax² + ay²) > imu_accel_spike_g` in
 *   `sensor_fusion.py`. Feeding raw accelerometer values there would inject a constant
 *   ~1 g of gravity on any tilted mount and trip a permanent false "IMU anomaly", so the
 *   gravity-free channel is the correct source.
 *
 * If the handset lacks the fused `TYPE_LINEAR_ACCELERATION` virtual sensor, gravity is
 * removed with a first-order low-pass estimate instead and [ImuState.hasLinearAcceleration]
 * reports `false`, so the UI can say the value is derived rather than measured.
 *
 * Samples arrive far faster than the 2 Hz uplink, so [consumePeakAccelG] keeps a peak-hold
 * between ticks: a 50 ms jolt would otherwise fall between two telemetry frames and never
 * reach the safety engine.
 */
class ImuSensor(private val context: Context) : SensorEventListener, ImuProvider {

  override val name: String = "Phone IMU"

  private val manager = context.getSystemService(Context.SENSOR_SERVICE) as? SensorManager

  private val accelerometer: Sensor? = manager?.getDefaultSensor(Sensor.TYPE_ACCELEROMETER)
  private val linearAccel: Sensor? = manager?.getDefaultSensor(Sensor.TYPE_LINEAR_ACCELERATION)
  private val gyroscope: Sensor? = manager?.getDefaultSensor(Sensor.TYPE_GYROSCOPE)
  private val magnetometer: Sensor? = manager?.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD)
  private val windowManager = context.getSystemService(Context.WINDOW_SERVICE) as? android.view.WindowManager

  private val _status = MutableStateFlow(if (accelerometer == null) SensorStatus.UNAVAILABLE else SensorStatus.WAITING)
  override val status: StateFlow<SensorStatus> = _status.asStateFlow()

  private val _reading = MutableStateFlow<ImuReading?>(null)
  override val reading: StateFlow<ImuReading?> = _reading.asStateFlow()

  override val hasAccelerometer: Boolean get() = accelerometer != null
  override val hasLinearAcceleration: Boolean get() = linearAccel != null
  override val hasGyroscope: Boolean get() = gyroscope != null || magnetometer != null

  private val _state = MutableStateFlow(initialState())
  val state: StateFlow<ImuState> = _state.asStateFlow()

  @Volatile private var pitchOffsetDeg = 0f

  @Volatile private var rollOffsetDeg = 0f

  // Latest raw samples (m/s²) — written on the sensor thread, read on the uplink thread.
  @Volatile private var rawAx = 0f

  @Volatile private var rawAy = 0f

  @Volatile private var rawAz = 0f

  @Volatile private var linAx = 0f

  @Volatile private var linAy = 0f

  @Volatile private var linAz = 0f

  @Volatile private var gyro: FloatArray? = null

  @Volatile private var magValues: FloatArray? = null
  @Volatile private var accelValues: FloatArray? = null
  @Volatile private var lastAzimuthRad: Float = 0f
  @Volatile private var lastAzimuthTimestampNs: Long = 0L
  @Volatile private var syntheticYawRateRps: Float = 0f
  @Volatile private var hasAzimuthInit: Boolean = false

  @Volatile private var lastSensorTimestampNs: Long = 0L

  @Volatile private var haveAccelSample = false

  /** The highest-magnitude linear-acceleration sample (axG, ayG, azG) since the last tick. */
  @Volatile private var peakSample: FloatArray? = null

  @Volatile private var peakAccelG = 0f

  /** Low-pass gravity estimate, only used when `TYPE_LINEAR_ACCELERATION` is missing. */
  private val gravityLp = floatArrayOf(0f, 0f, 0f)
  private var gravityLpPrimed = false

  private var running = false

  private fun initialState(): ImuState =
    ImuState(
      availability =
        when {
          manager == null || accelerometer == null -> Availability.UNAVAILABLE
          else -> Availability.WAITING
        },
      hasAccelerometer = accelerometer != null,
      hasLinearAcceleration = linearAccel != null,
      hasGyroscope = gyroscope != null || magnetometer != null,
      detail =
        when {
          accelerometer == null -> "No accelerometer on this device"
          gyroscope == null && magnetometer != null -> "Synthetic yaw rate derived from compass (no hardware gyro)"
          linearAccel == null -> "No linear-acceleration sensor — gravity removed in software"
          else -> ""
        },
    )

  override fun setMountOffsets(pitchDeg: Float, rollDeg: Float) {
    pitchOffsetDeg = pitchDeg
    rollOffsetDeg = rollDeg
  }

  private fun computeOrientationAngles(): Pair<Float, Float> {
    // Physical gravity detection: on a phone, gravity acts along the short edge (X)
    // ONLY when the device is mounted or held in landscape orientation.
    val isPhysicalLandscape = abs(rawAx) > 5.0f

    val (bodyAx, bodyAy, bodyAz) = when {
      rawAx < -5.0f -> Triple(rawAy, -rawAx, rawAz)
      rawAx > 5.0f  -> Triple(-rawAy, rawAx, rawAz)
      else -> Triple(rawAx, rawAy, rawAz)
    }

    val computedPitch = if (isPhysicalLandscape) {
      Units.RAD_TO_DEG * kotlin.math.atan2(bodyAz, kotlin.math.sqrt(bodyAx * bodyAx + bodyAy * bodyAy).coerceAtLeast(0.01f))
    } else {
      Units.pitchDeg(rawAx, rawAy, rawAz)
    }

    val computedRoll = if (isPhysicalLandscape) {
      Units.RAD_TO_DEG * kotlin.math.atan2(-bodyAx, bodyAy.coerceAtLeast(0.01f))
    } else {
      Units.rollDeg(rawAy, rawAz)
    }
    return Pair(computedPitch, computedRoll)
  }

  /**
   * Captures the current orientation as "level" and returns the new offsets.
   *
   * A phone clamped upright on a rover reads roll ≈ 90°, which would sit permanently above
   * the backend's 25° tilt limit. Calibration makes the mounted pose the zero point so the
   * transmitted tilt is the *vehicle's* tilt.
   */
  override fun calibrateLevel(): Pair<Float, Float>? {
    if (!haveAccelSample) return null
    val (rawPitch, rawRoll) = computeOrientationAngles()
    pitchOffsetDeg = rawPitch
    rollOffsetDeg = rawRoll
    publish()
    return rawPitch to rawRoll
  }

  override fun start() {
    val mgr = manager ?: return
    if (running) return
    running = true
    accelerometer?.let { mgr.registerListener(this, it, SensorManager.SENSOR_DELAY_UI) }
    linearAccel?.let { mgr.registerListener(this, it, SensorManager.SENSOR_DELAY_UI) }
    gyroscope?.let { mgr.registerListener(this, it, SensorManager.SENSOR_DELAY_UI) }
    magnetometer?.let { mgr.registerListener(this, it, SensorManager.SENSOR_DELAY_UI) }
  }

  override fun stop() {
    if (!running) return
    running = false
    manager?.unregisterListener(this)
    haveAccelSample = false
    gravityLpPrimed = false
    peakAccelG = 0f
    peakSample = null
    hasAzimuthInit = false
    lastAzimuthTimestampNs = 0L
    syntheticYawRateRps = 0f
    _state.value = initialState()
    _status.value = if (accelerometer == null) SensorStatus.UNAVAILABLE else SensorStatus.WAITING
    _reading.value = null
  }

  /**
   * The **actual measured sample** with the largest horizontal magnitude since the previous
   * call, as `(axG, ayG, azG)`, then resets.
   *
   * This is what gets transmitted rather than the most recent sample. The uplink runs at
   * 2 Hz while the sensor delivers ~16 Hz, so a 50 ms jolt would otherwise fall between two
   * frames and never reach the backend's `imu_accel_spike_g` rule. Nothing is synthesised —
   * it is one real sample from the interval, chosen by magnitude instead of by recency.
   */
  fun consumePeakSample(): Triple<Float, Float, Float>? {
    if (!haveAccelSample) return null
    val s = peakSample
    peakSample = null
    peakAccelG = 0f
    return if (s != null) Triple(s[0], s[1], s[2])
    else Triple(Units.mps2ToG(linAx), Units.mps2ToG(linAy), Units.mps2ToG(linAz))
  }

  override fun onSensorChanged(event: SensorEvent?) {
    val e = event ?: return
    lastSensorTimestampNs = android.os.SystemClock.elapsedRealtimeNanos()
    when (e.sensor.type) {
      Sensor.TYPE_ACCELEROMETER -> {
        rawAx = e.values[0]
        rawAy = e.values[1]
        rawAz = e.values[2]
        accelValues = floatArrayOf(rawAx, rawAy, rawAz)
        haveAccelSample = true
        if (linearAccel == null) {
          // alpha ≈ 0.8 — standard gravity low-pass from the SensorEvent documentation.
          if (!gravityLpPrimed) {
            gravityLp[0] = rawAx
            gravityLp[1] = rawAy
            gravityLp[2] = rawAz
            gravityLpPrimed = true
          } else {
            gravityLp[0] = 0.8f * gravityLp[0] + 0.2f * rawAx
            gravityLp[1] = 0.8f * gravityLp[1] + 0.2f * rawAy
            gravityLp[2] = 0.8f * gravityLp[2] + 0.2f * rawAz
          }
          linAx = rawAx - gravityLp[0]
          linAy = rawAy - gravityLp[1]
          linAz = rawAz - gravityLp[2]
          notePeak()
        }
        updateSyntheticYaw()
        publish()
      }
      Sensor.TYPE_MAGNETIC_FIELD -> {
        magValues = floatArrayOf(e.values[0], e.values[1], e.values[2])
        updateSyntheticYaw()
      }
      Sensor.TYPE_LINEAR_ACCELERATION -> {
        linAx = e.values[0]
        linAy = e.values[1]
        linAz = e.values[2]
        notePeak()
      }
      Sensor.TYPE_GYROSCOPE -> {
        gyro = floatArrayOf(e.values[0], e.values[1], e.values[2])
        if (haveAccelSample) {
          publish()
        }
      }
    }
  }

  private fun updateSyntheticYaw() {
    if (gyroscope != null) return
    val a = accelValues ?: return
    val m = magValues ?: return
    val r = FloatArray(9)
    val i = FloatArray(9)
    if (SensorManager.getRotationMatrix(r, i, a, m)) {
      val orientation = FloatArray(3)
      SensorManager.getOrientation(r, orientation)
      val azimuthRad = orientation[0]
      val nowNs = android.os.SystemClock.elapsedRealtimeNanos()

      if (hasAzimuthInit && lastAzimuthTimestampNs > 0L) {
        val dtSec = ((nowNs - lastAzimuthTimestampNs) / 1_000_000_000.0f).coerceIn(0.01f, 0.5f)
        var dAzimuth = azimuthRad - lastAzimuthRad
        while (dAzimuth > Math.PI) dAzimuth -= (2.0 * Math.PI).toFloat()
        while (dAzimuth < -Math.PI) dAzimuth += (2.0 * Math.PI).toFloat()

        val rawYawRateRps = if (abs(dAzimuth) < 0.02f) 0.0f else (dAzimuth / dtSec)
        syntheticYawRateRps = 0.85f * syntheticYawRateRps + 0.15f * rawYawRateRps
      } else {
        hasAzimuthInit = true
      }
      lastAzimuthRad = azimuthRad
      lastAzimuthTimestampNs = nowNs
    }
  }

  override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) = Unit

  private fun notePeak() {
    val axG = Units.mps2ToG(linAx)
    val ayG = Units.mps2ToG(linAy)
    val mag = sqrt(axG * axG + ayG * ayG)
    if (mag >= peakAccelG) {
      peakAccelG = mag
      peakSample = floatArrayOf(axG, ayG, Units.mps2ToG(linAz))
    }
  }

  private fun publish() {
    if (!haveAccelSample) return
    val (rawPitch, rawRoll) = computeOrientationAngles()
    val pitch = Units.applyOffset(rawPitch, pitchOffsetDeg)
    val roll = Units.applyOffset(rawRoll, rollOffsetDeg)

    val g = gyro
    val (bodyYawRateRad, bodyPitchRateRad, bodyRollRateRad) = when {
      g != null && abs(rawAx) > 5.0f -> {
        // Landscape mount: short edge (X) is vehicle yaw, long edge (Y) is vehicle pitch, screen normal (Z) is vehicle roll
        val sign = if (rawAx > 5.0f) 1.0f else -1.0f
        Triple(sign * g[0], sign * g[1], -g[2])
      }
      g != null -> {
        // Portrait or level orientation fallback
        Triple(g[2], g[0], g[1])
      }
      gyroscope == null -> {
        // Degraded compass derivative for yaw, zero for pitch/roll
        Triple(syntheticYawRateRps, 0.0f, 0.0f)
      }
      else -> Triple(0.0f, 0.0f, 0.0f)
    }

    val yawRate = Math.toDegrees(bodyYawRateRad.toDouble()).toFloat()

    val r = ImuReading(
      axG = Units.mps2ToG(linAx),
      ayG = Units.mps2ToG(linAy),
      azG = Units.mps2ToG(linAz),
      pitchDeg = pitch,
      rollDeg = roll,
      gyroX = bodyPitchRateRad,
      gyroY = bodyRollRateRad,
      gyroZ = bodyYawRateRad,
      timestamp = System.currentTimeMillis(),
      pitchOffsetDeg = pitchOffsetDeg,
      rollOffsetDeg = rollOffsetDeg,
      yawRateDps = yawRate,
      isAvailable = true,
      timestampNs = lastSensorTimestampNs,
    )
    _state.value =
      _state.value.copy(
        availability = Availability.AVAILABLE,
        reading = r,
        detail =
          if (gyroscope != null && linearAccel != null) "Hardware IMU active (6-DOF fusion)"
          else if (gyroscope != null) "Hardware gyro active (software gravity compensation)"
          else if (gyroscope == null && magnetometer != null) "Synthetic yaw rate derived from compass (no hardware gyro)"
          else if (linearAccel == null) "Gravity removed in software (no linear-accel sensor)"
          else if (abs(pitchOffsetDeg) > 0.01f || abs(rollOffsetDeg) > 0.01f)
            "Mount calibrated: pitch −${fmt(pitchOffsetDeg)}°, roll −${fmt(rollOffsetDeg)}°"
          else "",
      )
    _status.value = SensorStatus.ACTIVE
    _reading.value = r
  }

  /**
   * Evaluates whether the latest IMU sample is fresh relative to monotonic [nowNs].
   */
  fun isFresh(nowNs: Long, maxAgeNs: Long = 250_000_000L): Boolean {
    return lastSensorTimestampNs > 0L && (nowNs - lastSensorTimestampNs) in 0L..maxAgeNs
  }

  private fun fmt(v: Float) = String.format(java.util.Locale.US, "%.1f", v)
}
