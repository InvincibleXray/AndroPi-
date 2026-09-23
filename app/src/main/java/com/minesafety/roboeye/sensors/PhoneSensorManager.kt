package com.minesafety.roboeye.sensors

import android.content.Context
import com.minesafety.roboeye.core.model.PhoneSensorState
import com.minesafety.roboeye.sensors.imu.ImuProvider
import com.minesafety.roboeye.sensors.light.LightProvider
import com.minesafety.roboeye.sensors.light.PhoneLightSensor
import com.minesafety.roboeye.sensors.location.LocationProvider
import com.minesafety.roboeye.sensors.magnetometer.MagnetometerProvider
import com.minesafety.roboeye.sensors.magnetometer.PhoneMagnetometer
import com.minesafety.roboeye.sensors.rotation.PhoneRotationVector
import com.minesafety.roboeye.sensors.rotation.RotationVectorProvider
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.SupervisorJob
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.flow.update
import kotlinx.coroutines.launch

/**
 * Central orchestrator managing dynamic discovery, lifecycle, and aggregation of all phone sensors.
 *
 * Dynamic detection is guaranteed:
 * - If a sensor hardware is absent, status is UNAVAILABLE and value is null.
 * - Sensor values are NEVER fabricated or synthesized with fake random data.
 */
class PhoneSensorManager(
    val context: Context,
    val imu: ImuSensor = ImuSensor(context),
    val location: LocationSource = LocationSource(context),
    val light: LightProvider = PhoneLightSensor(context),
    val magnetometer: MagnetometerProvider = PhoneMagnetometer(context),
    val rotationVector: RotationVectorProvider = PhoneRotationVector(context),
    val battery: BatteryMonitor = BatteryMonitor(context),
    val health: DeviceHealth = DeviceHealth(context),
) {
    private val scope = CoroutineScope(SupervisorJob() + Dispatchers.Default)

    private val _sensorState = MutableStateFlow(PhoneSensorState())
    val sensorState: StateFlow<PhoneSensorState> = _sensorState.asStateFlow()

    private var running = false

    init {
        scope.launch {
            imu.reading.collect { reading ->
                _sensorState.update { current ->
                    current.copy(
                        imuStatus = imu.status.value,
                        imu = reading,
                        timestampMs = System.currentTimeMillis(),
                    )
                }
            }
        }

        scope.launch {
            light.reading.collect { reading ->
                _sensorState.update { current ->
                    current.copy(
                        lightStatus = light.status.value,
                        light = reading,
                    )
                }
            }
        }

        scope.launch {
            magnetometer.reading.collect { reading ->
                _sensorState.update { current ->
                    current.copy(
                        magnetometerStatus = magnetometer.status.value,
                        magnetometer = reading,
                    )
                }
            }
        }

        scope.launch {
            rotationVector.reading.collect { reading ->
                _sensorState.update { current ->
                    current.copy(
                        rotationStatus = rotationVector.status.value,
                        rotation = reading,
                    )
                }
            }
        }

        scope.launch {
            location.reading.collect { reading ->
                _sensorState.update { current ->
                    current.copy(
                        locationStatus = location.status.value,
                        location = reading,
                    )
                }
            }
        }

        scope.launch {
            battery.state.collect { b ->
                _sensorState.update { current ->
                    current.copy(
                        batteryPercent = b.percent,
                        isCharging = b.charging,
                    )
                }
            }
        }

        scope.launch {
            health.state.collect { h ->
                _sensorState.update { current ->
                    current.copy(
                        batteryTempC = h.reading?.batteryTempC,
                    )
                }
            }
        }
    }

    fun start() {
        if (running) return
        running = true
        imu.start()
        location.start()
        light.start()
        magnetometer.start()
        rotationVector.start()
        battery.start()
    }

    fun stop() {
        if (!running) return
        running = false
        imu.stop()
        location.stop()
        light.stop()
        magnetometer.stop()
        rotationVector.stop()
        battery.stop()
    }
}
