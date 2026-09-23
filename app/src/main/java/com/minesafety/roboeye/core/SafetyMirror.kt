package com.minesafety.roboeye.core

enum class MirrorLevel {
  NOMINAL,
  CAUTION,
  LOCAL_ESTOP,
}

data class MirrorVerdict(val level: MirrorLevel, val reasons: List<String>) {
  val isEStop: Boolean get() = level == MirrorLevel.LOCAL_ESTOP

  companion object {
    fun nominal() = MirrorVerdict(MirrorLevel.NOMINAL, emptyList())
  }
}

/**
 * Local safety mirror — a secondary indicator, never an authority.
 */
object SafetyMirror {

  const val MAX_TILT_DEG = 25.0f
  const val ACCEL_SPIKE_G = 2.5f
  const val ESTOP_DISTANCE_M = 0.5f
  const val REDUCE_SPEED_DISTANCE_M = 1.5f
  const val VISIBILITY_STOP = 0.10f
  const val VISIBILITY_LOW = 0.35f

  fun evaluate(
    pitchDeg: Float?,
    rollDeg: Float?,
    accelMagG: Float?,
    minObstacleM: Float?,
    bridgeConnected: Boolean,
    visibility: Float?,
  ): MirrorVerdict {
    val reasons = mutableListOf<String>()
    var level = MirrorLevel.NOMINAL

    fun escalate(to: MirrorLevel) {
      if (to.ordinal > level.ordinal) level = to
    }

    if (pitchDeg != null && rollDeg != null) {
      val tilt = Units.tiltDeg(pitchDeg, rollDeg)
      if (tilt > MAX_TILT_DEG) {
        escalate(MirrorLevel.LOCAL_ESTOP)
        reasons += "Extreme tilt ${fmt(tilt)}° > ${fmt(MAX_TILT_DEG)}°"
      }
    }

    if (accelMagG != null && accelMagG > ACCEL_SPIKE_G) {
      escalate(MirrorLevel.CAUTION)
      reasons += "Acceleration spike ${fmt(accelMagG)} g > ${fmt(ACCEL_SPIKE_G)} g"
    }

    if (bridgeConnected && minObstacleM != null) {
      when {
        minObstacleM < ESTOP_DISTANCE_M -> {
          escalate(MirrorLevel.LOCAL_ESTOP)
          reasons += "Obstacle ${fmt(minObstacleM)} m < ${fmt(ESTOP_DISTANCE_M)} m"
        }
        minObstacleM < REDUCE_SPEED_DISTANCE_M -> {
          escalate(MirrorLevel.CAUTION)
          reasons += "Obstacle ${fmt(minObstacleM)} m < ${fmt(REDUCE_SPEED_DISTANCE_M)} m"
        }
      }
    }

    if (visibility != null) {
      when {
        visibility < VISIBILITY_STOP -> {
          escalate(MirrorLevel.LOCAL_ESTOP)
          reasons += "Visibility ${fmt(visibility)} < ${fmt(VISIBILITY_STOP)}"
        }
        visibility < VISIBILITY_LOW -> {
          escalate(MirrorLevel.CAUTION)
          reasons += "Low visibility ${fmt(visibility)} < ${fmt(VISIBILITY_LOW)}"
        }
      }
    }

    return MirrorVerdict(level, reasons)
  }

  private fun fmt(v: Float): String = String.format(java.util.Locale.US, "%.2f", v)
}
