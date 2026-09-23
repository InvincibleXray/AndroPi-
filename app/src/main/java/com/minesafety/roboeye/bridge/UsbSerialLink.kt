package com.minesafety.roboeye.bridge

import android.app.PendingIntent
import android.content.BroadcastReceiver
import android.content.Context
import android.content.Intent
import android.content.IntentFilter
import android.hardware.usb.UsbDevice
import android.hardware.usb.UsbManager
import android.os.Build
import androidx.core.content.ContextCompat
import com.hoho.android.usbserial.driver.UsbSerialDriver
import com.hoho.android.usbserial.driver.UsbSerialPort
import com.hoho.android.usbserial.driver.UsbSerialProber
import com.hoho.android.usbserial.util.SerialInputOutputManager
import com.minesafety.roboeye.core.Logbook
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.flow.Flow
import kotlinx.coroutines.flow.MutableSharedFlow
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asSharedFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.launch
import kotlinx.coroutines.withContext

/**
 * Real ESP32 link over USB-OTG serial.
 *
 * Works with the USB-UART bridges found on ESP32 dev boards (CP2102, CH340/CH9102, FTDI) and
 * with native-CDC boards, via `usb-serial-for-android`'s default prober. The wire protocol is
 * newline-delimited JSON in both directions — see [BridgeCodec].
 *
 * Two details that matter on real hardware:
 *
 * * **DTR/RTS are asserted** after open. CDC-ACM devices (and some CP210x configurations)
 *   transmit nothing until DTR is raised, which otherwise looks exactly like a dead board.
 * * **The partial-line buffer is bounded** ([MAX_PARTIAL_CHARS]). A board that resets mid-frame,
 *   or one whose baud rate does not match, can emit megabytes without ever sending `\n`;
 *   an unbounded accumulator would grow until the process is killed.
 */
class UsbSerialLink(
  private val context: Context,
  private val scope: CoroutineScope,
  private val logbook: Logbook,
) : RoverLink {

  override val name: String = "ESP32 over USB-OTG"

  private val _status = MutableStateFlow<LinkStatus>(LinkStatus.Disconnected)
  override val status: StateFlow<LinkStatus> = _status.asStateFlow()

  private val _lines = MutableSharedFlow<String>(extraBufferCapacity = 64)
  override val lines: Flow<String> = _lines.asSharedFlow()

  private val usbManager: UsbManager? =
    context.getSystemService(Context.USB_SERVICE) as? UsbManager

  private var port: UsbSerialPort? = null
  private var io: SerialInputOutputManager? = null
  private val partial = StringBuilder()
  private var receiverRegistered = false

  /** Human-readable device name, exposed for the Bridge screen. */
  @Volatile var deviceName: String? = null
    private set

  private val permissionReceiver =
    object : BroadcastReceiver() {
      override fun onReceive(ctx: Context?, intent: Intent?) {
        if (intent?.action != ACTION_USB_PERMISSION) return
        val granted = intent.getBooleanExtra(UsbManager.EXTRA_PERMISSION_GRANTED, false)
        if (granted) {
          logbook.info(TAG, "USB permission granted — opening port")
          scope.launch { open() }
        } else {
          logbook.warn(TAG, "USB permission denied by user")
          _status.value = LinkStatus.NoPermission
        }
      }
    }

  private fun ensureReceiver() {
    if (receiverRegistered) return
    receiverRegistered = true
    ContextCompat.registerReceiver(
      context,
      permissionReceiver,
      IntentFilter(ACTION_USB_PERMISSION),
      ContextCompat.RECEIVER_NOT_EXPORTED,
    )
  }

  /** Lists attached devices a driver exists for — used by the Bridge screen. */
  fun availableDevices(): List<String> {
    val mgr = usbManager ?: return emptyList()
    return UsbSerialProber.getDefaultProber().findAllDrivers(mgr).map { describe(it.device) }
  }

  override suspend fun open(): Boolean {
    val mgr = usbManager
    if (mgr == null) {
      _status.value = LinkStatus.Error("This device has no USB host support")
      return false
    }
    close()
    ensureReceiver()
    _status.value = LinkStatus.Connecting

    val driver: UsbSerialDriver? =
      UsbSerialProber.getDefaultProber().findAllDrivers(mgr).firstOrNull()
    if (driver == null) {
      _status.value = LinkStatus.NoDevice
      return false
    }
    val device = driver.device
    deviceName = describe(device)

    if (!mgr.hasPermission(device)) {
      _status.value = LinkStatus.NoPermission
      requestPermission(device)
      return false
    }

    val connection = mgr.openDevice(device)
    if (connection == null) {
      _status.value = LinkStatus.Error("Could not open USB device (in use or permission lost)")
      return false
    }

    val serialPort = driver.ports.firstOrNull()
    if (serialPort == null) {
      _status.value = LinkStatus.Error("Driver exposes no serial port")
      return false
    }

    return withContext(Dispatchers.IO) {
      try {
        serialPort.open(connection)
        serialPort.setParameters(BAUD_RATE, 8, UsbSerialPort.STOPBITS_1, UsbSerialPort.PARITY_NONE)
        // Non-fatal: FTDI/CP210x accept these, some drivers throw UnsupportedOperationException.
        runCatching { serialPort.setDTR(true) }
        runCatching { serialPort.setRTS(true) }
        port = serialPort
        partial.setLength(0)
        io =
          SerialInputOutputManager(serialPort, ioListener).also {
            it.readTimeout = READ_TIMEOUT_MS
            it.start()
          }
        _status.value = LinkStatus.Connected(deviceName ?: "ESP32")
        logbook.info(TAG, "Serial port open: ${deviceName} @ $BAUD_RATE baud 8N1")
        true
      } catch (e: Exception) {
        runCatching { serialPort.close() }
        port = null
        val msg = e.message ?: e::class.java.simpleName
        _status.value = LinkStatus.Error(msg)
        logbook.error(TAG, "Serial open failed: $msg")
        false
      }
    }
  }

  override suspend fun write(line: String): Boolean {
    val p = port ?: return false
    val payload = if (line.endsWith("\n")) line else "$line\n"
    return withContext(Dispatchers.IO) {
      runCatching { p.write(payload.toByteArray(Charsets.US_ASCII), WRITE_TIMEOUT_MS) }
        .onFailure { logbook.warn(TAG, "Serial write failed: ${it.message}") }
        .isSuccess
    }
  }

  override fun close() {
    runCatching { io?.stop() }
    io = null
    runCatching { port?.close() }
    port = null
    partial.setLength(0)
    if (_status.value is LinkStatus.Connected) _status.value = LinkStatus.Disconnected
  }

  fun shutdown() {
    close()
    if (receiverRegistered) {
      receiverRegistered = false
      runCatching { context.unregisterReceiver(permissionReceiver) }
    }
  }

  private val ioListener =
    object : SerialInputOutputManager.Listener {
      override fun onNewData(data: ByteArray?) {
        val bytes = data ?: return
        // The ESP32 sketch emits ASCII JSON; ignore anything non-ASCII from line noise.
        val text = String(bytes, Charsets.US_ASCII)
        synchronized(partial) {
          for (ch in text) {
            when (ch) {
              '\n' -> emitLine()
              '\r' -> Unit
              else -> {
                if (partial.length >= MAX_PARTIAL_CHARS) {
                  logbook.warn(
                    TAG,
                    "Discarding ${partial.length} bytes with no line terminator — check baud rate",
                  )
                  partial.setLength(0)
                }
                partial.append(ch)
              }
            }
          }
        }
      }

      override fun onRunError(e: Exception?) {
        val msg = e?.message ?: "serial read error"
        logbook.error(TAG, "Serial link dropped: $msg")
        _status.value = LinkStatus.Error(msg)
        close()
      }
    }

  private fun emitLine() {
    val line = partial.toString()
    partial.setLength(0)
    if (line.isBlank()) return
    // tryEmit keeps the serial reader thread non-blocking; a full buffer drops the oldest
    // consumer's chance at this line rather than stalling the USB pump.
    if (!_lines.tryEmit(line)) {
      scope.launch { _lines.emit(line) }
    }
  }

  private fun requestPermission(device: UsbDevice) {
    val mgr = usbManager ?: return
    val flags =
      if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
        // FLAG_MUTABLE is required: the system fills in EXTRA_PERMISSION_GRANTED.
        PendingIntent.FLAG_UPDATE_CURRENT or PendingIntent.FLAG_MUTABLE
      } else {
        PendingIntent.FLAG_UPDATE_CURRENT
      }
    val intent =
      PendingIntent.getBroadcast(
        context,
        0,
        Intent(ACTION_USB_PERMISSION).setPackage(context.packageName),
        flags,
      )
    logbook.info(TAG, "Requesting USB permission for ${describe(device)}")
    runCatching { mgr.requestPermission(device, intent) }
      .onFailure { _status.value = LinkStatus.Error(it.message ?: "USB permission request failed") }
  }

  private fun describe(device: UsbDevice): String {
    val product = runCatching { device.productName }.getOrNull()
    val vid = String.format("%04X", device.vendorId)
    val pid = String.format("%04X", device.productId)
    return if (product.isNullOrBlank()) "USB $vid:$pid" else "$product ($vid:$pid)"
  }

  private companion object {
    const val TAG = "Bridge"
    const val ACTION_USB_PERMISSION = "com.minesafety.roboeye.USB_PERMISSION"

    /** Matches `Serial.begin(115200)` in both the reference firmware and the companion sketch. */
    const val BAUD_RATE = 115_200
    const val READ_TIMEOUT_MS = 200
    const val WRITE_TIMEOUT_MS = 500
    const val MAX_PARTIAL_CHARS = 4_096
  }
}
