package com.minesafety.roboeye.net

import com.minesafety.roboeye.core.BackendConnectionStatus
import com.minesafety.roboeye.core.ImuReading
import com.minesafety.roboeye.core.RadarConfig
import com.minesafety.roboeye.core.RoadGeometryState
import com.minesafety.roboeye.core.TrackedObject
import java.io.IOException
import java.util.concurrent.TimeUnit
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.withContext
import kotlinx.serialization.decodeFromString
import kotlinx.serialization.encodeToString
import kotlinx.serialization.json.Json
import okhttp3.MediaType.Companion.toMediaType
import okhttp3.OkHttpClient
import okhttp3.Request
import okhttp3.RequestBody.Companion.toRequestBody

class BackendRadarClient {

    private val json = Json {
        ignoreUnknownKeys = true
        encodeDefaults = true
        isLenient = true
    }

    private val jsonMediaType = "application/json; charset=utf-8".toMediaType()

    private val httpClient: OkHttpClient = OkHttpClient.Builder()
        .connectTimeout(3, TimeUnit.SECONDS)
        .readTimeout(4, TimeUnit.SECONDS)
        .writeTimeout(4, TimeUnit.SECONDS)
        .retryOnConnectionFailure(true)
        .build()

    private val _connectionStatus = MutableStateFlow(BackendConnectionStatus.OFFLINE)
    val connectionStatus: StateFlow<BackendConnectionStatus> = _connectionStatus.asStateFlow()

    @Volatile private var cachedToken: String? = null

    fun setCachedToken(token: String?) {
        cachedToken = token
        if (token != null) {
            _connectionStatus.value = BackendConnectionStatus.CONNECTED
        } else {
            _connectionStatus.value = BackendConnectionStatus.OFFLINE
        }
    }

    // There is deliberately no "last successful uplink" timestamp here. One used to be
    // written on every accepted POST and read by nothing. It looked like staleness tracking
    // without being any: `connectionStatus` is already re-evaluated on every single attempt
    // — CONNECTED on an accepted post, ERROR on a rejection, OFFLINE on a transport failure
    // — so there is no window in which it can describe an uplink that has stopped working.
    // A field that implies a freshness check nobody performs is worse than no field, because
    // the next reader assumes the check exists.

    suspend fun authenticate(config: RadarConfig): Result<String> = withContext(Dispatchers.IO) {
        // No credentials means there is nothing to try. Since RadarConfig now ships blank rather
        // than with the backend's seeded `operator`/`operator123` account, an unconfigured node
        // would otherwise POST an empty username on every uplink tick and take a 401 each time.
        // UNAUTHORIZED is the honest state: the backend is not refusing us, we simply have no
        // identity to present — and it is the same state a wrong password produces, which is what
        // the operator needs to see either way.
        if (!config.hasCredentials) {
            _connectionStatus.value = BackendConnectionStatus.UNAUTHORIZED
            return@withContext Result.failure(
                IOException("No operator credentials configured — enter them in Radar settings")
            )
        }
        _connectionStatus.value = BackendConnectionStatus.CONNECTING
        runCatching {
            val url = "${config.backendUrl}/api/auth/login"
            val loginPayload = json.encodeToString(
                LoginRequestDto(config.operatorUsername, config.operatorPassword)
            )

            val request = Request.Builder()
                .url(url)
                .post(loginPayload.toRequestBody(jsonMediaType))
                .build()

            httpClient.newCall(request).execute().use { response ->
                val body = response.body?.string().orEmpty()
                if (response.code == 401 || response.code == 403) {
                    _connectionStatus.value = BackendConnectionStatus.UNAUTHORIZED
                    throw IOException("Invalid operator credentials: HTTP ${response.code}")
                }
                if (!response.isSuccessful) {
                    _connectionStatus.value = BackendConnectionStatus.ERROR
                    throw IOException("Auth failed HTTP ${response.code}: $body")
                }

                val tokenResp = json.decodeFromString<TokenResponseDto>(body)
                cachedToken = tokenResp.accessToken
                _connectionStatus.value = BackendConnectionStatus.CONNECTED
                tokenResp.accessToken
            }
        }.onFailure {
            if (_connectionStatus.value != BackendConnectionStatus.UNAUTHORIZED) {
                _connectionStatus.value = BackendConnectionStatus.OFFLINE
            }
        }
    }

    suspend fun postPerceptionPacket(
        config: RadarConfig,
        trackedObjects: List<TrackedObject>,
        visibilityScore: Float?,
        imuReading: ImuReading? = null,
        roadGeometry: RoadGeometryState? = null,
    ): Result<VisionRadarAckDto> {
        val score = visibilityScore ?: 0.5f
        val packet = RadarPacketBuilder.buildPacket(
            config = config,
            trackedObjects = trackedObjects,
            visibilityScore = score,
            roadGeometry = roadGeometry,
        )
        return postRadarScan(config, packet)
    }

    suspend fun postRadarScan(config: RadarConfig, packet: VisionRadarPacketDto): Result<VisionRadarAckDto> =
        withContext(Dispatchers.IO) {
            runCatching {
                // Ensure we have a valid token
                val token = cachedToken ?: authenticate(config).getOrThrow()

                val url = "${config.backendUrl}/api/perception/vision-radar"
                val payload = json.encodeToString(packet)

                val request = Request.Builder()
                    .url(url)
                    .header("Authorization", "Bearer $token")
                    .post(payload.toRequestBody(jsonMediaType))
                    .build()

                httpClient.newCall(request).execute().use { response ->
                    val body = response.body?.string().orEmpty()

                    if (response.code == 401) {
                        // Token might have expired, invalidate and retry once
                        cachedToken = null
                        val newToken = authenticate(config).getOrThrow()
                        val retryReq = Request.Builder()
                            .url(url)
                            .header("Authorization", "Bearer $newToken")
                            .post(payload.toRequestBody(jsonMediaType))
                            .build()
                        httpClient.newCall(retryReq).execute().use { retryResp ->
                            val retryBody = retryResp.body?.string().orEmpty()
                            if (!retryResp.isSuccessful) {
                                throw IOException("Retry failed HTTP ${retryResp.code}: $retryBody")
                            }
                            _connectionStatus.value = BackendConnectionStatus.CONNECTED
                            json.decodeFromString<VisionRadarAckDto>(retryBody)
                        }
                    } else if (!response.isSuccessful) {
                        _connectionStatus.value = BackendConnectionStatus.ERROR
                        throw IOException("Ingest failed HTTP ${response.code}: $body")
                    } else {
                        _connectionStatus.value = BackendConnectionStatus.CONNECTED
                        json.decodeFromString<VisionRadarAckDto>(body)
                    }
                }
            }.onFailure {
                if (_connectionStatus.value != BackendConnectionStatus.UNAUTHORIZED) {
                    _connectionStatus.value = BackendConnectionStatus.OFFLINE
                }
            }
        }

    suspend fun checkHealth(baseUrl: String): Result<Pair<HealthInfoDto, Long>> = withContext(Dispatchers.IO) {
        runCatching {
            val start = System.currentTimeMillis()
            val request = Request.Builder()
                .url("$baseUrl/api/health")
                .get()
                .build()

            httpClient.newCall(request).execute().use { response ->
                val body = response.body?.string().orEmpty()
                if (!response.isSuccessful) throw IOException("HTTP ${response.code}")
                val info = json.decodeFromString<HealthInfoDto>(body)
                val latency = System.currentTimeMillis() - start
                info to latency
            }
        }
    }
}
