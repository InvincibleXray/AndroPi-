package com.minesafety.roboeye.net

import com.minesafety.roboeye.core.Logbook
import java.io.IOException
import java.util.concurrent.TimeUnit
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.withContext
import kotlinx.serialization.decodeFromString
import kotlinx.serialization.encodeToString
import okhttp3.MediaType.Companion.toMediaType
import okhttp3.OkHttpClient
import okhttp3.Request
import okhttp3.RequestBody.Companion.toRequestBody

/**
 * HTTP half of the backend contract.
 */
class BackendClient(private val logbook: Logbook) {

  val http: OkHttpClient =
    OkHttpClient.Builder()
      .connectTimeout(CONNECT_TIMEOUT_S, TimeUnit.SECONDS)
      .readTimeout(READ_TIMEOUT_S, TimeUnit.SECONDS)
      .writeTimeout(READ_TIMEOUT_S, TimeUnit.SECONDS)
      .pingInterval(PING_INTERVAL_S, TimeUnit.SECONDS)
      .retryOnConnectionFailure(true)
      .build()

  suspend fun login(baseUrl: String, username: String, password: String): Result<LoginResponse> =
    request(
      url = "$baseUrl/api/auth/login",
      body = RoverJson.encodeToString(LoginRequest(username, password)),
    ) { text ->
      RoverJson.decodeFromString<LoginResponse>(text)
    }

  suspend fun health(baseUrl: String): Result<Pair<HealthInfo, Long>> =
    withContext(Dispatchers.IO) {
      val started = System.currentTimeMillis()
      runCatching {
          val req = Request.Builder().url("$baseUrl/api/health").get().build()
          http.newCall(req).execute().use { resp ->
            val text = resp.body?.string().orEmpty()
            if (!resp.isSuccessful) throw IOException("HTTP ${resp.code}")
            val info = RoverJson.decodeFromString<HealthInfo>(text)
            info to (System.currentTimeMillis() - started)
          }
        }
        .onFailure { logbook.warn(TAG, "Health probe failed: ${describe(it)}") }
    }

  suspend fun postTelemetry(
    baseUrl: String,
    token: String,
    vehicleId: String,
    frame: TelemetryFrame,
  ): Result<IngestAck> =
    request(
      url = "$baseUrl/api/vehicles/$vehicleId/telemetry",
      body = RoverJson.encodeToString(frame),
      token = token,
    ) { text ->
      RoverJson.decodeFromString<IngestAck>(text)
    }

  suspend fun postVisionFrame(
    baseUrl: String,
    token: String?,
    vehicleId: String,
    jpeg: ByteArray,
  ): Result<Int> =
    withContext(Dispatchers.IO) {
      runCatching {
        val builder =
          Request.Builder()
            .url("$baseUrl/api/vision/frame?vehicle_id=$vehicleId")
            .post(jpeg.toRequestBody(JPEG))
        token?.let { builder.header("Authorization", "Bearer $it") }
        http.newCall(builder.build()).execute().use { resp ->
          if (!resp.isSuccessful) throw IOException("HTTP ${resp.code}")
          resp.body?.close()
          jpeg.size
        }
      }
    }

  private suspend fun <T> request(
    url: String,
    body: String,
    token: String? = null,
    parse: (String) -> T,
  ): Result<T> =
    withContext(Dispatchers.IO) {
      runCatching {
        val builder = Request.Builder().url(url).post(body.toRequestBody(JSON))
        token?.let { builder.header("Authorization", "Bearer $it") }
        http.newCall(builder.build()).execute().use { resp ->
          val text = resp.body?.string().orEmpty()
          if (!resp.isSuccessful) {
            throw IOException("HTTP ${resp.code}${detailOf(text)}")
          }
          parse(text)
        }
      }
    }

  private fun detailOf(text: String): String {
    if (text.isBlank()) return ""
    val detail =
      runCatching {
          val obj = RoverJson.parseToJsonElement(text)
          obj.toString().takeIf { it.length < 200 }
        }
        .getOrNull()
    return detail?.let { " — $it" } ?: ""
  }

  companion object {
    private const val TAG = "HTTP"
    private const val CONNECT_TIMEOUT_S = 4L
    private const val READ_TIMEOUT_S = 8L
    private const val PING_INTERVAL_S = 5L
    private val JSON = "application/json; charset=utf-8".toMediaType()
    private val JPEG = "image/jpeg".toMediaType()

    fun describe(t: Throwable): String = t.message ?: t::class.java.simpleName
  }
}
