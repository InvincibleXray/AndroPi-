package com.minesafety.roboeye.sensors

import android.content.Context
import android.net.ConnectivityManager
import android.net.Network
import android.net.NetworkCapabilities
import com.minesafety.roboeye.core.NetworkState
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow

/**
 * Tracks the default network for Robo Eye.
 */
class NetworkMonitor(private val context: Context) {

  private val manager = context.getSystemService(Context.CONNECTIVITY_SERVICE) as? ConnectivityManager

  private val _state = MutableStateFlow(NetworkState())
  val state: StateFlow<NetworkState> = _state.asStateFlow()

  private var registered = false

  private val callback =
    object : ConnectivityManager.NetworkCallback() {
      override fun onAvailable(network: Network) = refresh()

      override fun onLost(network: Network) = refresh()

      override fun onCapabilitiesChanged(network: Network, caps: NetworkCapabilities) = refresh()
    }

  fun start() {
    val mgr = manager ?: return
    if (registered) return
    registered = true
    runCatching { mgr.registerDefaultNetworkCallback(callback) }
    refresh()
  }

  fun stop() {
    if (!registered) return
    registered = false
    runCatching { manager?.unregisterNetworkCallback(callback) }
  }

  fun refresh() {
    val mgr = manager
    if (mgr == null) {
      _state.value = NetworkState(online = false, transport = "unavailable")
      return
    }
    val active = mgr.activeNetwork
    val caps = active?.let { runCatching { mgr.getNetworkCapabilities(it) }.getOrNull() }
    if (caps == null) {
      _state.value = NetworkState(online = false, transport = "none")
      return
    }
    val transport =
      when {
        caps.hasTransport(NetworkCapabilities.TRANSPORT_WIFI) -> "Wi-Fi"
        caps.hasTransport(NetworkCapabilities.TRANSPORT_CELLULAR) -> "Cellular"
        caps.hasTransport(NetworkCapabilities.TRANSPORT_ETHERNET) -> "Ethernet"
        else -> "other"
      }
    val hasLink =
      caps.hasTransport(NetworkCapabilities.TRANSPORT_WIFI) ||
        caps.hasTransport(NetworkCapabilities.TRANSPORT_CELLULAR) ||
        caps.hasTransport(NetworkCapabilities.TRANSPORT_ETHERNET)
    val internet = caps.hasCapability(NetworkCapabilities.NET_CAPABILITY_INTERNET)
    _state.value =
      NetworkState(
        online = hasLink,
        transport = if (internet) transport else "$transport (LAN only)",
      )
  }
}
