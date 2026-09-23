package com.minesafety.roboeye.sensors.light

import android.content.Context
import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import com.minesafety.roboeye.core.model.LightReading
import com.minesafety.roboeye.core.model.SensorStatus
import com.minesafety.roboeye.sensors.base.SensorProvider
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow

/**
 * Interface and Android SensorManager implementation for ambient light sensing.
 */
interface LightProvider : SensorProvider<LightReading>

class PhoneLightSensor(context: Context) : LightProvider, SensorEventListener {

    override val name: String = "Ambient Light Sensor"

    private val sensorManager = context.getSystemService(Context.SENSOR_SERVICE) as? SensorManager
    private val lightSensor: Sensor? = sensorManager?.getDefaultSensor(Sensor.TYPE_LIGHT)

    private val _status = MutableStateFlow(
        if (lightSensor == null) SensorStatus.UNAVAILABLE else SensorStatus.WAITING
    )
    override val status: StateFlow<SensorStatus> = _status.asStateFlow()

    private val _reading = MutableStateFlow<LightReading?>(null)
    override val reading: StateFlow<LightReading?> = _reading.asStateFlow()

    private var running = false

    override fun start() {
        val mgr = sensorManager ?: return
        val sensor = lightSensor ?: return
        if (running) return
        running = true
        _status.value = SensorStatus.WAITING
        mgr.registerListener(this, sensor, SensorManager.SENSOR_DELAY_NORMAL)
    }

    override fun stop() {
        if (!running) return
        running = false
        sensorManager?.unregisterListener(this)
        _status.value = if (lightSensor == null) SensorStatus.UNAVAILABLE else SensorStatus.WAITING
        _reading.value = null
    }

    override fun onSensorChanged(event: SensorEvent?) {
        if (event == null || event.values.isEmpty()) return
        val lux = event.values[0]
        _status.value = SensorStatus.ACTIVE
        _reading.value = LightReading(
            lux = lux,
            timestampNs = event.timestamp,
        )
    }

    override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) {
        // No-op
    }
}
