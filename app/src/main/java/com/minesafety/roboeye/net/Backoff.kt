package com.minesafety.roboeye.net

import kotlin.random.Random

/**
 * Bounded exponential backoff.
 *
 * Delay grows base * 2^n, capped at maxMs, with +-20% jitter.
 */
class Backoff(
  private val baseMs: Long = 1_000L,
  private val maxMs: Long = 15_000L,
  val maxAttempts: Int = 8,
  private val random: Random = Random.Default,
) {

  var attempts: Int = 0
    private set

  val exhausted: Boolean get() = attempts >= maxAttempts

  fun reset() {
    attempts = 0
  }

  /** Returns the delay for the next attempt, or null when the retry budget is spent. */
  fun nextDelayMs(): Long? {
    if (exhausted) return null
    val exponent = attempts.coerceAtMost(EXPONENT_CAP)
    attempts++
    val raw = baseMs shl exponent
    val capped = raw.coerceAtMost(maxMs)
    val jitter = (capped * JITTER_FRACTION).toLong().coerceAtLeast(1L)
    return (capped - jitter + random.nextLong(2 * jitter + 1)).coerceAtLeast(MIN_DELAY_MS)
  }

  private companion object {
    const val EXPONENT_CAP = 20
    const val JITTER_FRACTION = 0.2f
    const val MIN_DELAY_MS = 250L
  }
}
