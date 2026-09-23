package com.minesafety.roboeye.bridge

import kotlinx.coroutines.flow.Flow
import kotlinx.coroutines.flow.StateFlow

/** Connection state of a [RoverLink]. */
sealed interface LinkStatus {
  data object Disconnected : LinkStatus

  /** No compatible USB serial device is attached. */
  data object NoDevice : LinkStatus

  /** A device is attached but the user has not granted USB permission yet. */
  data object NoPermission : LinkStatus

  data object Connecting : LinkStatus

  data class Connected(val deviceName: String) : LinkStatus

  data class Error(val message: String) : LinkStatus

  val label: String
    get() =
      when (this) {
        Disconnected -> "DISCONNECTED"
        NoDevice -> "NO DEVICE"
        NoPermission -> "PERMISSION REQUIRED"
        Connecting -> "CONNECTING"
        is Connected -> "CONNECTED"
        is Error -> "ERROR"
      }
}

/**
 * Transport abstraction for the ESP32 companion link.
 */
interface RoverLink {

  /** Short human name shown on the Bridge screen. */
  val name: String

  val status: StateFlow<LinkStatus>

  /** Raw inbound lines, exactly as received (no parsing, no filtering). */
  val lines: Flow<String>

  /** Opens the link. Returns true when [status] became [LinkStatus.Connected]. */
  suspend fun open(): Boolean

  /** Writes one line (a trailing `\n` is added if missing). Returns false on failure. */
  suspend fun write(line: String): Boolean

  fun close()
}
