package com.minesafety.roboeye.net

import com.minesafety.roboeye.core.Logbook
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.Job
import kotlinx.coroutines.channels.BufferOverflow
import kotlinx.coroutines.channels.Channel
import kotlinx.coroutines.isActive
import kotlinx.coroutines.launch
import okhttp3.MediaType.Companion.toMediaType
import okhttp3.OkHttpClient
import okhttp3.Request
import okhttp3.RequestBody.Companion.toRequestBody

/**
 * Lightweight frame container carrying the encoded image and stream metadata.
 */
data class FramePayload(
  val jpeg: ByteArray,
  val seq: Long,
  val timestamp: Long,
  val width: Int,
  val height: Int,
  val rotation: Int = 0,
)

/**
 * Continuous Live Video Transport for the Android Rover Node.
 *
 * Designed specifically for low-latency live camera streaming over LAN to the FastAPI backend.
 *
 * ### Key Properties
 * 1. **Latest-Frame Preference**: Uses a single-slot bounded [Channel] with [BufferOverflow.DROP_OLDEST].
 *    If the network or backend is slower than the camera analysis rate, older frames are
 *    discarded immediately so the driver view never accumulates historical lag.
 * 2. **Rich Frame Metadata**: Every frame is tagged with sequence number, UTC epoch timestamp,
 *    pixel resolution, and camera orientation.
 * 3. **Connection Reuse**: Operates over OkHttp's shared HTTP/1.1 connection pool with persistent
 *    keep-alive, minimizing per-frame TCP handshake latency.
 */
class VideoTransport(
  private val scope: CoroutineScope,
  private val httpClient: OkHttpClient,
  private val logbook: Logbook,
) {

  /** Invoked after a successful frame delivery with byte count, completion timestamp, and round-trip latency. */
  var onUploaded: ((bytes: Int, at: Long, latencyMs: Long) -> Unit)? = null

  /** Invoked when a frame was dropped due to network or buffer backpressure. */
  var onDropped: (() -> Unit)? = null

  private var queue: Channel<FramePayload>? = null
  private var worker: Job? = null

  private var baseUrl: String = ""
  @Volatile private var token: String? = null
  private var vehicleId: String = ""

  @Volatile var framesSent: Long = 0L
    private set

  @Volatile var framesDropped: Long = 0L
    private set

  @Volatile var failures: Long = 0L
    private set

  private val fpsWindow = ArrayDeque<Long>()

  fun start(baseUrl: String, token: String?, vehicleId: String) {
    stop()
    this.baseUrl = baseUrl.trimEnd('/')
    this.token = token
    this.vehicleId = vehicleId

    // Capacity 1 with DROP_OLDEST guarantees we always deliver the freshest available frame
    val ch = Channel<FramePayload>(capacity = 1, onBufferOverflow = BufferOverflow.DROP_OLDEST)
    queue = ch

    worker =
      scope.launch(Dispatchers.IO) {
        val mediaType = "image/jpeg".toMediaType()

        for (frame in ch) {
          if (!isActive) break

          val currentToken = token
          if (currentToken.isNullOrBlank()) {
            // Standalone mode: skip network delivery without error
            continue
          }

          val url = "$baseUrl/api/vision/frame?vehicle_id=$vehicleId"
          val body = frame.jpeg.toRequestBody(mediaType)
          val requestBuilder =
            Request.Builder()
              .url(url)
              .post(body)
              .header("X-Frame-Seq", frame.seq.toString())
              .header("X-Timestamp", frame.timestamp.toString())
              .header("X-Width", frame.width.toString())
              .header("X-Height", frame.height.toString())
              .header("X-Rotation", frame.rotation.toString())
              .header("X-Camera-Id", "front_rgb")
              .header("Authorization", "Bearer $currentToken")

          val startTime = System.currentTimeMillis()
          try {
            httpClient.newCall(requestBuilder.build()).execute().use { response ->
              val latency = System.currentTimeMillis() - startTime
              if (response.isSuccessful) {
                framesSent++
                noteTx(System.currentTimeMillis())
                onUploaded?.invoke(frame.jpeg.size, System.currentTimeMillis(), latency)
              } else if (response.code == 401 || response.code == 403) {
                failures++
                logbook.warn(TAG, "Video transport auth failed (HTTP ${response.code}) — token invalid or expired")
                // Pause further uplink attempts until new token is supplied
                this@VideoTransport.token = null
              } else {
                failures++
                if (failures % FAILURE_LOG_EVERY == 1L) {
                  logbook.warn(TAG, "Video transport HTTP error ${response.code}: ${response.message}")
                }
              }
            }
          } catch (e: Exception) {
            failures++
            if (failures % FAILURE_LOG_EVERY == 1L) {
              logbook.warn(TAG, "Video transport network failure: ${e.message ?: e::class.java.simpleName}")
            }
          }
        }
      }

    if (token.isNullOrBlank()) {
      logbook.info(TAG, "VideoTransport standing by for $vehicleId (Standalone mode — sign in via Settings to enable live uplink)")
    } else {
      logbook.info(TAG, "VideoTransport started for $vehicleId -> $baseUrl (Authenticated live uplink)")
    }
  }

  fun updateToken(newToken: String?) {
    this.token = newToken
    if (!newToken.isNullOrBlank()) {
      logbook.info(TAG, "Video transport authenticated — streaming uplink active")
    } else {
      logbook.info(TAG, "Video transport standing by (standalone mode)")
    }
  }

  /**
   * Enqueues a frame for transmission.
   *
   * Drops old frames if the channel is congested, guaranteeing freshest frame delivery.
   */
  fun offer(payload: FramePayload): Boolean {
    // If not authenticated, do not enqueue frames for network transmission
    if (token.isNullOrBlank()) return false

    val ch = queue ?: return false
    val result = ch.trySend(payload)
    if (!result.isSuccess) {
      framesDropped++
      onDropped?.invoke()
      if (framesDropped % DROP_LOG_EVERY == 1L) {
        logbook.warn(TAG, "Video transport backpressure: $framesDropped frame(s) dropped")
      }
      return false
    }
    return true
  }

  fun currentStreamingFps(): Float {
    synchronized(fpsWindow) {
      if (fpsWindow.size < 2) return 0f
      val span = fpsWindow.last() - fpsWindow.first()
      return if (span <= 0) 0f else (fpsWindow.size - 1) * 1000f / span
    }
  }

  private fun noteTx(now: Long) {
    synchronized(fpsWindow) {
      fpsWindow.addLast(now)
      while (fpsWindow.size > FPS_WINDOW) fpsWindow.removeFirst()
    }
  }

  fun stop() {
    queue?.close()
    queue = null
    worker?.cancel()
    worker = null
    synchronized(fpsWindow) { fpsWindow.clear() }
  }

  private companion object {
    const val TAG = "VideoTransport"
    const val DROP_LOG_EVERY = 15L
    const val FAILURE_LOG_EVERY = 10L
    const val FPS_WINDOW = 12
  }
}
