package com.minesafety.roboeye.perception.benchmark

import android.content.Context
import android.os.Build
import android.os.PowerManager

/**
 * System monitor collecting non-invasive device, memory, and thermal state telemetry.
 */
object DeviceSystemMonitor {

    fun getDeviceManufacturer(): String = Build.MANUFACTURER ?: "UNKNOWN"

    fun getDeviceModel(): String = Build.MODEL ?: "UNKNOWN"

    fun getAndroidVersion(): String = Build.VERSION.RELEASE ?: "UNKNOWN"

    fun getApiLevel(): Int = Build.VERSION.SDK_INT

    fun getAppVersion(context: Context?): String {
        return try {
            if (context != null) {
                val pInfo = context.packageManager.getPackageInfo(context.packageName, 0)
                pInfo.versionName ?: "1.0.0"
            } else {
                "1.0.0"
            }
        } catch (_: Throwable) {
            "1.0.0"
        }
    }

    /**
     * Java/Kotlin heap memory usage in Megabytes.
     */
    fun getMemoryUsageMb(): Float {
        val runtime = Runtime.getRuntime()
        val usedBytes = runtime.totalMemory() - runtime.freeMemory()
        return (usedBytes / (1024f * 1024f))
    }

    /**
     * Java/Kotlin heap max memory ceiling in Megabytes.
     */
    fun getMemoryMaxMb(): Float {
        val runtime = Runtime.getRuntime()
        return (runtime.maxMemory() / (1024f * 1024f))
    }

    /**
     * Thermal status via Android PowerManager (API 29+).
     * Reports NOT_AVAILABLE on older APIs or if unreadable.
     */
    fun getThermalStatus(context: Context?): String {
        if (context == null || Build.VERSION.SDK_INT < Build.VERSION_CODES.Q) {
            return "NOT_AVAILABLE"
        }
        return try {
            val powerManager = context.getSystemService(Context.POWER_SERVICE) as? PowerManager
            when (powerManager?.currentThermalStatus) {
                PowerManager.THERMAL_STATUS_NONE -> "NORMAL"
                PowerManager.THERMAL_STATUS_LIGHT -> "LIGHT"
                PowerManager.THERMAL_STATUS_MODERATE -> "MODERATE"
                PowerManager.THERMAL_STATUS_SEVERE -> "SEVERE"
                PowerManager.THERMAL_STATUS_CRITICAL -> "CRITICAL"
                PowerManager.THERMAL_STATUS_EMERGENCY -> "EMERGENCY"
                PowerManager.THERMAL_STATUS_SHUTDOWN -> "SHUTDOWN"
                else -> "NOT_AVAILABLE"
            }
        } catch (_: Throwable) {
            "NOT_AVAILABLE"
        }
    }
}
