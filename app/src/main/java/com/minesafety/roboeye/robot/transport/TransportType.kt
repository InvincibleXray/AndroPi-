package com.minesafety.roboeye.robot.transport

enum class TransportType(val label: String) {
    USB("USB-OTG Serial"),
    WIFI("Wi-Fi Direct / UDP"),
    BLUETOOTH("Bluetooth Serial / BLE"),
    MOCK("Simulated Chassis Loopback"),
}
