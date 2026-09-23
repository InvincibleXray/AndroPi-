package com.minesafety.roboeye.core

import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow

enum class LogLevel {
  INFO,
  WARNING,
  ERROR,
}

data class LogEntry(
  val id: Long,
  val timestamp: Long,
  val level: LogLevel,
  val tag: String,
  val message: String,
) {
  val clock: String get() = Units.clockLocal(timestamp)
}

/**
 * Bounded, in-memory event log surfaced on the Logs screen.
 *
 * Deliberately capped at [CAPACITY] entries with oldest-first eviction: the node runs
 * unattended for hours on a 2 GB Redmi 6A, so an unbounded log would be a slow memory
 * leak. Nothing is written to disk.
 */
class Logbook(private val capacity: Int = CAPACITY) {

  private val lock = Any()
  private var nextId = 1L
  private val entries = ArrayDeque<LogEntry>(capacity)

  private val _log = MutableStateFlow<List<LogEntry>>(emptyList())
  val log: StateFlow<List<LogEntry>> = _log.asStateFlow()

  private val _droppedCount = MutableStateFlow(0)

  /** Number of entries evicted because the ring was full (shown on the Logs screen). */
  val droppedCount: StateFlow<Int> = _droppedCount.asStateFlow()

  fun info(tag: String, message: String) = add(LogLevel.INFO, tag, message)

  fun warn(tag: String, message: String) = add(LogLevel.WARNING, tag, message)

  fun error(tag: String, message: String) = add(LogLevel.ERROR, tag, message)

  fun add(level: LogLevel, tag: String, message: String) {
    val snapshot: List<LogEntry>
    var dropped = 0
    synchronized(lock) {
      entries.addLast(LogEntry(nextId++, System.currentTimeMillis(), level, tag, message))
      while (entries.size > capacity) {
        entries.removeFirst()
        dropped++
      }
      // Newest first — that is the useful order while debugging a live rover.
      snapshot = entries.toList().asReversed()
    }
    if (dropped > 0) _droppedCount.value = _droppedCount.value + dropped
    _log.value = snapshot
  }

  fun clear() {
    synchronized(lock) { entries.clear() }
    _droppedCount.value = 0
    _log.value = emptyList()
  }

  companion object {
    const val CAPACITY = 300
  }
}
