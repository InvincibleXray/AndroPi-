package com.minesafety.roboeye.net

import com.minesafety.roboeye.core.Logbook
import com.minesafety.roboeye.core.SocketState
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Job
import kotlinx.coroutines.delay
import kotlinx.coroutines.flow.MutableSharedFlow
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.SharedFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asSharedFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.launch
import kotlinx.serialization.decodeFromString
import kotlinx.serialization.encodeToString
import okhttp3.OkHttpClient
import okhttp3.Request
import okhttp3.Response
import okhttp3.WebSocket
import okhttp3.WebSocketListener

/**
 * Telemetry uplink and command downlink over `WS /ws/ingest/{vehicle_id}?token=…`.
 *
 * The token travels as a query parameter because a WebSocket handshake from a browser or
 * OkHttp cannot carry a custom `Authorization` header through every proxy — this is the same
 * pattern the backend route expects. The project's tokens are base64url-encoded
 * (`core/security.py`), so no percent-encoding is required.
 *
 * Reconnection is **bounded** (see [Backoff]). When the budget is spent the socket parks in
 * [SocketState.RetryExhausted] and waits for an explicit operator retry rather than
 * hammering an unreachable host forever. An auth rejection is treated as terminal, not
 * transient: retrying it cannot help. Every other backend complaint is per-frame — the
 * backend keeps the socket open, and so do we.
 */
class IngestSocket(
  private val client: OkHttpClient,
  private val scope: CoroutineScope,
  private val logbook: Logbook,
) {

  private val _state = MutableStateFlow<SocketState>(SocketState.Idle)
  val state: StateFlow<SocketState> = _state.asStateFlow()
  val isConnected: Boolean get() = _state.value is SocketState.Connected

  private val _commands = MutableSharedFlow<CommandFrame>(extraBufferCapacity = COMMAND_BUFFER)
  val commands: SharedFlow<CommandFrame> = _commands.asSharedFlow()

  /**
   * The backend's latest advisory, or `null` before the first one arrives.
   *
   * A [StateFlow] rather than a [SharedFlow] because advisories are **latest-wins state, not
   * events**: one arrives per accepted uplink frame (≈2 Hz), each supersedes the last, and a
   * missed one is of no consequence. A buffered event flow would give this a queue that could
   * overflow and a backlog worth replaying — neither is true of an operating picture.
   */
  private val _advisory = MutableStateFlow<AdvisoryFrame?>(null)
  val advisory: StateFlow<AdvisoryFrame?> = _advisory.asStateFlow()

  private var socket: WebSocket? = null
  private var reconnectJob: Job? = null
  private var backoff = Backoff()

  private var url: String = ""
  private var token: String = ""
  private var vehicleId: String = ""

  /** True once the caller asked to stop — suppresses all reconnection. */
  @Volatile private var stopped = true

  @Volatile private var framesDropped = 0L

  @Volatile private var framesRejected = 0L

  /** Frames discarded because OkHttp's send queue was over [MAX_QUEUE_BYTES]. */
  val droppedFrames: Long get() = framesDropped

  /** Frames the backend answered with a non-terminal `error` frame (bad JSON / schema). */
  val rejectedFrames: Long get() = framesRejected

  fun connect(wsUrl: String, bearerToken: String, vehicle: String) {
    url = wsUrl
    token = bearerToken
    vehicleId = vehicle
    stopped = false
    backoff = Backoff()
    openNow()
  }

  /** Operator-initiated retry after the bounded budget was exhausted. */
  fun retry() {
    if (url.isBlank()) return
    stopped = false
    backoff = Backoff()
    openNow()
  }

  fun close() {
    stopped = true
    reconnectJob?.cancel()
    reconnectJob = null
    runCatching { socket?.close(NORMAL_CLOSURE, "node stopped") }
    socket = null
    _state.value = SocketState.Idle
    // A stopped node has no operating picture. Leaving the last advisory on screen would
    // present the backend's final opinion as its current one. Age-out covers a *dropped*
    // link; this covers a deliberate stop, where there is nothing to age.
    _advisory.value = null
  }

  /**
   * Sends one telemetry frame.
   *
   * Returns false when the socket is not connected, or when the outbound queue is already
   * over [MAX_QUEUE_BYTES] — a stalled TCP connection must not be allowed to accumulate
   * minutes of stale telemetry that would be delivered as a burst of history. Dropping the
   * newest frame at 2 Hz costs 500 ms of resolution; queueing it costs correctness.
   */
  fun send(frame: TelemetryFrame): Boolean {
    val ws = socket ?: return false
    if (_state.value !is SocketState.Connected) return false
    if (ws.queueSize() > MAX_QUEUE_BYTES) {
      framesDropped++
      if (framesDropped % DROP_LOG_EVERY == 1L) {
        logbook.warn(TAG, "Uplink stalled — dropped $framesDropped frame(s) to keep the queue bounded")
      }
      return false
    }
    return runCatching { ws.send(RoverJson.encodeToString(frame)) }.getOrDefault(false)
  }

  private fun openNow() {
    reconnectJob?.cancel()
    runCatching { socket?.cancel() }
    socket = null
    _state.value = SocketState.Connecting
    val request =
      Request.Builder().url(if (token.isBlank()) url else "$url?token=$token").build()
    socket = client.newWebSocket(request, listener)
  }

  private val listener =
    object : WebSocketListener() {

      override fun onOpen(webSocket: WebSocket, response: Response) {
        logbook.info(TAG, "Socket open — waiting for backend hello")
        // Stay in Connecting until the backend's `hello` proves the route accepted us.
        // A 101 upgrade alone does not mean auth or provider mode passed.
      }

      override fun onMessage(webSocket: WebSocket, text: String) {
        val frame =
          runCatching { RoverJson.decodeFromString<InboundFrame>(text) }
            .onFailure { logbook.warn(TAG, "Unparseable frame from backend: ${text.take(120)}") }
            .getOrNull() ?: return

        when {
          frame.isError -> {
            val reason = frame.error ?: "backend rejected the ingest session"
            // Only an auth rejection is terminal. Per-frame complaints ("invalid json",
            // "invalid reading: …") leave the socket open on the backend side, so tearing the
            // uplink down over one bad frame would take the node offline — and the dashboard
            // would silently fall back to the simulated tick. Log and keep pushing.
            if (isTerminalIngestError(reason)) {
              logbook.error(TAG, "Backend refused ingest: $reason")
              stopped = true
              _state.value = SocketState.Failed(reason)
              runCatching { webSocket.close(NORMAL_CLOSURE, "rejected") }
            } else {
              framesRejected++
              logbook.warn(TAG, "Backend rejected a frame ($reason) — uplink stays open")
            }
          }
          frame.isHello -> {
            backoff.reset()
            _state.value =
              SocketState.Connected(frame.vehicleId ?: vehicleId, frame.tickHz)
            logbook.info(
              TAG,
              "Ingest session established for ${frame.vehicleId ?: vehicleId}" +
                (frame.tickHz?.let { " @ ${it} Hz backend tick" } ?: ""),
            )
          }
          frame.isCommand -> {
            val cmd = CommandFrame.from(frame, System.currentTimeMillis()) ?: return
            logbook.info(TAG, "Command received: ${cmd.label}")
            if (!_commands.tryEmit(cmd)) {
              logbook.warn(TAG, "Command buffer full — dropped ${cmd.type}")
            }
          }
          frame.isAdvisory -> {
            // Decoded a second time against the advisory's own shape. `InboundFrame` is the
            // routing envelope and deliberately does not carry twenty display fields; a failed
            // decode here must not disturb the uplink, so it is logged and dropped.
            val advisory =
              runCatching { RoverJson.decodeFromString<AdvisoryFrame>(text) }
                .onFailure { logbook.warn(TAG, "Unparseable advisory: ${text.take(120)}") }
                .getOrNull() ?: return
            _advisory.value = advisory
          }
          else -> logbook.info(TAG, "Backend frame: ${frame.kind.ifBlank { text.take(80) }}")
        }
      }

      override fun onFailure(webSocket: WebSocket, t: Throwable, response: Response?) {
        val reason = response?.let { "HTTP ${it.code}" } ?: BackendClient.describe(t)
        socket = null
        if (stopped) {
          _state.value = SocketState.Idle
          return
        }
        // A 403/1008 arriving as an HTTP failure is also terminal.
        if (response?.code == 401 || response?.code == 403) {
          stopped = true
          _state.value = SocketState.Failed("$reason — operator token rejected")
          logbook.error(TAG, "Ingest auth rejected ($reason)")
          return
        }
        scheduleReconnect(reason)
      }

      override fun onClosed(webSocket: WebSocket, code: Int, reason: String) {
        socket = null
        if (stopped) {
          _state.value = SocketState.Idle
          return
        }
        scheduleReconnect(if (reason.isBlank()) "closed ($code)" else "$reason ($code)")
      }
    }

  private fun scheduleReconnect(reason: String) {
    val delayMs = backoff.nextDelayMs()
    if (delayMs == null) {
      logbook.error(
        TAG,
        "Reconnect budget exhausted after ${backoff.attempts} attempts — last error: $reason",
      )
      _state.value = SocketState.RetryExhausted(backoff.attempts, reason)
      return
    }
    _state.value = SocketState.Reconnecting(backoff.attempts, backoff.maxAttempts, delayMs)
    logbook.warn(
      TAG,
      "Uplink lost ($reason) — retry ${backoff.attempts}/${backoff.maxAttempts} in ${delayMs} ms",
    )
    reconnectJob?.cancel()
    reconnectJob =
      scope.launch {
        delay(delayMs)
        if (!stopped) openNow()
      }
  }

  private companion object {
    const val TAG = "Uplink"
    const val NORMAL_CLOSURE = 1000

    /**
     * ~32 KB of queued telemetry ≈ 60 frames. Beyond that the link is not slow, it is
     * broken, and the newest reading matters more than the backlog.
     */
    const val MAX_QUEUE_BYTES = 32L * 1024L
    const val COMMAND_BUFFER = 16
    const val DROP_LOG_EVERY = 20L
  }
}

/**
 * Classifies a backend `error` frame as terminal (stop and wait for the operator) or
 * per-frame (log and keep the uplink open).
 *
 * Terminal means only one thing here: the operator token was rejected. `ws.py` answers a
 * malformed frame with `{"type":"error", …}` and *keeps the socket open*, so anything else
 * must not take the node offline — a node that quits over one bad frame hands the dashboard
 * back to the simulator without saying so.
 *
 * Matching on message text is loose, but the backend closes the socket with `1008` right
 * after an auth rejection anyway, so a miss costs at most one wasted reconnect while a false
 * positive would strand a working node. The asymmetry decides the direction.
 */
internal fun isTerminalIngestError(reason: String): Boolean {
  val lower = reason.lowercase(java.util.Locale.US)
  return TERMINAL_ERROR_MARKERS.any { lower.contains(it) }
}

/** Substrings of `ws.py`'s auth rejection ("unauthorized — operator token required"). */
private val TERMINAL_ERROR_MARKERS = listOf("unauthorized", "operator token", "forbidden")
