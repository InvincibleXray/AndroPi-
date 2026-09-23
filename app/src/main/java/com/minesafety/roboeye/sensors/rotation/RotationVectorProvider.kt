package com.minesafety.roboeye.sensors.rotation

import android.content.Context
import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import com.minesafety.roboeye.core.model.RotationVectorReading
import com.minesafety.roboeye.core.model.SensorStatus
import com.minesafety.roboeye.sensors.base.SensorProvider
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow

/**
 * Interface and Android SensorManager implementation for fused orientation vector.
 */
interface RotationVectorProvider : SensorProvider<RotationVectorReading>

class PhoneRotationVector(context: Context) : RotationVectorProvider, SensorEventListener {

    override val name: String = "Rotation Vector Sensor"

    private val sensorManager = context.getSystemService(Context.SENSOR_SERVICE) as? SensorManager
    private val rotSensor: Sensor? = sensorManager?.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR)

    private val _status = MutableStateFlow(
        if (rotSensor == null) SensorStatus.UNAVAILABLE else SensorStatus.WAITING
    )
    override val status: StateFlow<SensorStatus> = _status.asStateFlow()

    private val _reading = MutableStateFlow<RotationVectorReading?>(null)
    override val reading: StateFlow<RotationVectorReading?> = _reading.asStateFlow()

    private val rotationMatrix = FloatArray(9)
    private val orientationAngles = FloatArray(3)
    private var running = false

    override fun start() {
        val mgr = sensorManager ?: return
        val sensor = rotSensor ?: return
        if (running) return
        running = true
        _status.value = SensorStatus.WAITING
        mgr.registerListener(this, sensor, SensorManager.SENSOR_DELAY_UI)
    }

    override fun stop() {
        if (!running) return
        running = false
        sensorManager?.unregisterListener(this)
        _status.value = if (rotSensor == null) SensorStatus.UNAVAILABLE else SensorStatus.WAITING
        _reading.value = null
    }

    override fun onSensorChanged(event: SensorEvent?) {
        if (event == null || event.values.size < 4) return
        val qx = event.values[0]
        val qy = event.values[1]
        val qz = event.values[2]
        val qw = event.values[3]

        SensorManager.getRotationMatrixFromVector(rotationMatrix, event.values)
        SensorManager.getOrientation(rotationMatrix, orientationAngles)
        val azimuthDeg = ((Math.toDegrees(orientationAngles[0].toDouble()) + 360.0) % 360.0).toFloat()

        _status.value = SensorStatus.ACTIVE
        _reading.value = RotationVectorReading(
            qx = qx,
            qy = qy,
            qz = qz,
            qw = qw,
            headingDeg = azimuthDeg,
            timestampNs = event.timestamp,
        )
    }

    override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) {
        // No-op
    }
}
