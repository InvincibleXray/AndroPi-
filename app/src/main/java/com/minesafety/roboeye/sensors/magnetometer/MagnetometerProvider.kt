package com.minesafety.roboeye.sensors.magnetometer

import android.content.Context
import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import com.minesafety.roboeye.core.model.MagnetometerReading
import com.minesafety.roboeye.core.model.SensorStatus
import com.minesafety.roboeye.sensors.base.SensorProvider
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlin.math.atan2

/**
 * Interface and Android SensorManager implementation for magnetic compass sensing.
 */
interface MagnetometerProvider : SensorProvider<MagnetometerReading>

class PhoneMagnetometer(context: Context) : MagnetometerProvider, SensorEventListener {

    override val name: String = "Magnetic Field Sensor"

    private val sensorManager = context.getSystemService(Context.SENSOR_SERVICE) as? SensorManager
    private val magSensor: Sensor? = sensorManager?.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD)

    private val _status = MutableStateFlow(
        if (magSensor == null) SensorStatus.UNAVAILABLE else SensorStatus.WAITING
    )
    override val status: StateFlow<SensorStatus> = _status.asStateFlow()

    private val _reading = MutableStateFlow<MagnetometerReading?>(null)
    override val reading: StateFlow<MagnetometerReading?> = _reading.asStateFlow()

    private var running = false

    override fun start() {
        val mgr = sensorManager ?: return
        val sensor = magSensor ?: return
        if (running) return
        running = true
        _status.value = SensorStatus.WAITING
        mgr.registerListener(this, sensor, SensorManager.SENSOR_DELAY_UI)
    }

    override fun stop() {
        if (!running) return
        running = false
        sensorManager?.unregisterListener(this)
        _status.value = if (magSensor == null) SensorStatus.UNAVAILABLE else SensorStatus.WAITING
        _reading.value = null
    }

    override fun onSensorChanged(event: SensorEvent?) {
        if (event == null || event.values.size < 3) return
        val mx = event.values[0]
        val my = event.values[1]
        val mz = event.values[2]
        val azimuthRad = atan2(-my.toDouble(), mx.toDouble())
        val azimuthDeg = ((Math.toDegrees(azimuthRad) + 360.0) % 360.0).toFloat()

        _status.value = SensorStatus.ACTIVE
        _reading.value = MagnetometerReading(
            mxUt = mx,
            myUt = my,
            mzUt = mz,
            azimuthDeg = azimuthDeg,
            timestampNs = event.timestamp,
        )
    }

    override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) {
        // No-op
    }
}
