package com.minesafety.roboeye.bridge

import kotlinx.serialization.json.Json
import kotlinx.serialization.json.JsonObject
import kotlinx.serialization.json.JsonPrimitive
import kotlinx.serialization.json.buildJsonObject
import kotlinx.serialization.json.floatOrNull
import kotlinx.serialization.json.jsonPrimitive

/**
 * One decoded telemetry line from the ESP32.
 */
data class BridgeTelemetry(
  val ultrasonicFront: Float? = null,
  val ultrasonicLeft: Float? = null,
  val ultrasonicRight: Float? = null,
  val irLeft: Int? = null,
  val irCenter: Int? = null,
  val irRight: Int? = null,
  val imuAxG: Float? = null,
  val imuAyG: Float? = null,
  val imuAzG: Float? = null,
  val imuPitchDeg: Float? = null,
  val imuRollDeg: Float? = null,
  val estop: Boolean? = null,
  val uptimeMs: Long? = null,
  val timestamp: Long = 0L,
) {
  val minObstacleM: Float?
    get() = listOfNotNull(ultrasonicFront, ultrasonicLeft, ultrasonicRight).minOrNull()

  val hasRanges: Boolean
    get() = ultrasonicFront != null || ultrasonicLeft != null || ultrasonicRight != null

  val hasIr: Boolean get() = irLeft != null || irCenter != null || irRight != null

  val hasImu: Boolean get() = imuPitchDeg != null && imuRollDeg != null
}

data class BridgeReject(val line: String, val reason: String)

sealed interface BridgeParse {
  data class Ok(val telemetry: BridgeTelemetry) : BridgeParse
  data class Notice(val text: String) : BridgeParse
  data class Rejected(val reason: String) : BridgeParse
}

/**
 * Newline-delimited JSON codec for the phone <-> ESP32 serial link.
 */
object BridgeCodec {

  private val json = Json {
    ignoreUnknownKeys = true
    isLenient = true
  }

  const val MIN_RANGE_M = 0.02f
  const val MAX_RANGE_M = 4.5f
  private const val MAX_ACCEL_G = 16f

  fun parse(line: String, now: Long): BridgeParse {
    val trimmed = line.trim()
    if (trimmed.isEmpty()) return BridgeParse.Rejected("empty line")
    if (!trimmed.startsWith("{")) return BridgeParse.Notice(trimmed)

    val obj =
      try {
        json.parseToJsonElement(trimmed) as? JsonObject
          ?: return BridgeParse.Rejected("not a JSON object")
      } catch (e: Exception) {
        return BridgeParse.Rejected("malformed JSON: ${e.message?.take(60) ?: "parse error"}")
      }

    if (obj.containsKey("type") && !obj.containsKey("ultrasonic_front")) {
      return BridgeParse.Notice(trimmed)
    }

    val t =
      BridgeTelemetry(
        ultrasonicFront = obj.range("ultrasonic_front"),
        ultrasonicLeft = obj.range("ultrasonic_left"),
        ultrasonicRight = obj.range("ultrasonic_right"),
        irLeft = obj.digital("ir_left"),
        irCenter = obj.digital("ir_center"),
        irRight = obj.digital("ir_right"),
        imuAxG = obj.accel("imu_ax"),
        imuAyG = obj.accel("imu_ay"),
        imuAzG = obj.accel("imu_az"),
        imuPitchDeg = obj.angle("imu_pitch"),
        imuRollDeg = obj.angle("imu_roll"),
        estop = obj.bool("estop") ?: obj.bool("emergency_stop"),
        uptimeMs = obj.long("uptime_ms"),
        timestamp = now,
      )

    if (!t.hasRanges && !t.hasIr && !t.hasImu) {
      return BridgeParse.Rejected("no usable sensor fields")
    }
    return BridgeParse.Ok(t)
  }

  fun encodeCommand(
    type: String,
    value: Float?,
    seq: Long? = null,
    timestampMs: Long? = null,
  ): String {
    val obj = buildJsonObject {
      put("command", JsonPrimitive(type))
      put("type", JsonPrimitive(type))
      put("value", JsonPrimitive(value ?: 0f))
      if (seq != null) put("seq", JsonPrimitive(seq))
      if (timestampMs != null) put("timestamp_ms", JsonPrimitive(timestampMs))
    }
    return obj.toString()
  }

  private fun JsonObject.num(key: String): Float? {
    val p = this[key] as? JsonPrimitive ?: return null
    if (p.isString && p.content.equals("null", ignoreCase = true)) return null
    val v = p.floatOrNull ?: return null
    return if (v.isNaN() || v.isInfinite()) null else v
  }

  private fun JsonObject.range(key: String): Float? =
    num(key)?.takeIf { it in MIN_RANGE_M..MAX_RANGE_M }

  private fun JsonObject.digital(key: String): Int? =
    num(key)?.let { if (it >= 0.5f) 1 else 0 }

  private fun JsonObject.accel(key: String): Float? =
    num(key)?.takeIf { kotlin.math.abs(it) <= MAX_ACCEL_G }

  private fun JsonObject.angle(key: String): Float? = num(key)?.takeIf { kotlin.math.abs(it) <= 180f }

  private fun JsonObject.bool(key: String): Boolean? {
    val p = this[key] as? JsonPrimitive ?: return null
    p.content.lowercase().let {
      if (it == "true" || it == "1") return true
      if (it == "false" || it == "0") return false
    }
    return null
  }

  private fun JsonObject.long(key: String): Long? =
    (this[key] as? JsonPrimitive)?.jsonPrimitive?.content?.toDoubleOrNull()?.toLong()
}
