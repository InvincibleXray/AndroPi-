package com.minesafety.roboeye.net

import java.net.DatagramPacket
import java.net.DatagramSocket
import java.net.InetAddress
import java.net.SocketTimeoutException
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.withContext
import org.json.JSONObject

/**
 * Discovers the Mine Safety Rover FastAPI backend server on the local Wi-Fi network (LAN)
 * by listening to broadcast beacons and sending probe packets on UDP port 50007.
 */
object ServerDiscovery {
  const val DISCOVERY_PORT = 50007

  data class DiscoveredServer(
    val ip: String,
    val port: Int,
    val service: String,
    val version: String
  )

  suspend fun discover(timeoutMs: Int = 3000): DiscoveredServer? = withContext(Dispatchers.IO) {
    var socket: DatagramSocket? = null
    try {
      socket = DatagramSocket().apply {
        broadcast = true
        soTimeout = timeoutMs
      }

      // Send active probe packet
      val probeJson = "{\"cmd\":\"DISCOVER_SERVER\",\"service\":\"sih26007\"}"
      val probeBytes = probeJson.toByteArray(Charsets.UTF_8)
      val broadcastAddr = InetAddress.getByName("255.255.255.255")
      val probePacket = DatagramPacket(probeBytes, probeBytes.size, broadcastAddr, DISCOVERY_PORT)
      socket.send(probePacket)

      // Listen for incoming beacon or probe reply
      val buf = ByteArray(1024)
      val recvPacket = DatagramPacket(buf, buf.size)
      val deadline = System.currentTimeMillis() + timeoutMs

      while (System.currentTimeMillis() < deadline) {
        val remaining = (deadline - System.currentTimeMillis()).toInt().coerceAtLeast(100)
        socket.soTimeout = remaining
        socket.receive(recvPacket)

        val text = String(recvPacket.data, 0, recvPacket.length, Charsets.UTF_8)
        val json = JSONObject(text)
        val service = json.optString("service")
        val ip = json.optString("server_ip")
        val port = json.optInt("port", 8000)

        if (service == "sih26007" && ip.isNotBlank()) {
          return@withContext DiscoveredServer(
            ip = ip,
            port = port,
            service = service,
            version = json.optString("version", "1.0.0")
          )
        }
      }
      null
    } catch (e: SocketTimeoutException) {
      null
    } catch (e: Exception) {
      null
    } finally {
      socket?.close()
    }
  }
}
