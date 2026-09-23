package com.minesafety.roboeye.robot.transport

/**
 * Universal connection state of a [RobotTransport].
 */
sealed interface TransportStatus {
    data object Disconnected : TransportStatus
    data object Connecting : TransportStatus
    data class Connected(val endpointName: String) : TransportStatus
    data class Error(val message: String) : TransportStatus

    val isConnected: Boolean get() = this is Connected

    val label: String
        get() = when (this) {
            Disconnected -> "DISCONNECTED"
            Connecting -> "CONNECTING"
            is Connected -> "CONNECTED ($endpointName)"
            is Error -> "ERROR ($message)"
        }
}
