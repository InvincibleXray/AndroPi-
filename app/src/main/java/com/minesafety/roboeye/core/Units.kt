package com.minesafety.roboeye.core

import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale
import java.util.TimeZone
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.roundToInt
import kotlin.math.sqrt

/**
 * Unit conversions, math, and time formatting shared by sensors, perception, and networking.
 */
object Units {
    const val RAD_TO_DEG = (180.0 / PI).toFloat()
    const val DEG_TO_RAD = (PI / 180.0).toFloat()
    const val GRAVITY = 9.80665f

    /** m/s² → g */
    fun mps2ToG(v: Float): Float = v / GRAVITY

    /**
     * Pitch in degrees from a gravity/acceleration vector.
     * atan2(-ax, sqrt(ay² + az²)) * (180 / PI)
     */
    fun pitchDeg(ax: Float, ay: Float, az: Float): Float =
        atan2(-ax, sqrt(ay * ay + az * az)) * RAD_TO_DEG

    /** Roll in degrees: atan2(ay, az) * (180 / PI) */
    fun rollDeg(ay: Float, az: Float): Float =
        atan2(ay, az) * RAD_TO_DEG

    fun tiltDeg(pitchDeg: Float, rollDeg: Float): Float =
        maxOf(abs(pitchDeg), abs(rollDeg))

    fun accelMagnitudeG(axG: Float, ayG: Float): Float =
        sqrt(axG * axG + ayG * ayG)

    fun normaliseDeg(deg: Float): Float {
        var d = deg % 360f
        if (d > 180f) d -= 360f
        if (d <= -180f) d += 360f
        return d
    }

    fun normalizeDeg(deg: Float): Float = normaliseDeg(deg)

    fun applyOffset(rawDeg: Float, offsetDeg: Float): Float =
        normaliseDeg(rawDeg - offsetDeg)

    fun clamp(v: Float, lo: Float = 0f, hi: Float = 1f): Float =
        maxOf(lo, minOf(hi, v))

    fun clamp01(v: Float): Float = clamp(v, 0f, 1f)

    fun round1(v: Float): Float = (v * 10f).roundToInt() / 10f

    fun round2(v: Float): Float = (v * 100f).roundToInt() / 100f

    fun isoUtc(epochMillis: Long): String {
        val fmt = SimpleDateFormat("yyyy-MM-dd'T'HH:mm:ss.SSS", Locale.US)
        fmt.timeZone = TimeZone.getTimeZone("UTC")
        return fmt.format(Date(epochMillis)) + "Z"
    }

    fun clockLocal(epochMillis: Long): String =
        SimpleDateFormat("HH:mm:ss", Locale.US).format(Date(epochMillis))

    fun ageLabel(epochMillis: Long?, nowMillis: Long): String {
        if (epochMillis == null || epochMillis <= 0L) return "—"
        val ms = (nowMillis - epochMillis).coerceAtLeast(0L)
        return when {
            ms < 1_000 -> "${ms} ms ago"
            ms < 60_000 -> String.format(Locale.US, "%.1f s ago", ms / 1000f)
            else -> "${ms / 60_000} min ago"
        }
    }
}
