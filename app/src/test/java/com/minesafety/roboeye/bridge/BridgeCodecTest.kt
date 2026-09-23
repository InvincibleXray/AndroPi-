package com.minesafety.roboeye.bridge

import org.junit.Assert.assertEquals
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertTrue
import org.junit.Test

class BridgeCodecTest {

    @Test
    fun parse_validTelemetryLine_returnsOkWithFields() {
        val line = """{"ultrasonic_front": 1.42, "ultrasonic_left": 0.85, "ir_center": 1, "imu_pitch": 3.5, "imu_roll": -1.2}"""
        val now = 1000L
        val result = BridgeCodec.parse(line, now)

        assertTrue(result is BridgeParse.Ok)
        val t = (result as BridgeParse.Ok).telemetry
        assertEquals(1.42f, t.ultrasonicFront ?: 0f, 0.001f)
        assertEquals(0.85f, t.ultrasonicLeft ?: 0f, 0.001f)
        assertEquals(1, t.irCenter)
        assertEquals(3.5f, t.imuPitchDeg ?: 0f, 0.001f)
        assertEquals(-1.2f, t.imuRollDeg ?: 0f, 0.001f)
        assertEquals(now, t.timestamp)
    }

    @Test
    fun parse_emptyOrBlankLine_returnsRejected() {
        val result1 = BridgeCodec.parse("", 1000L)
        assertTrue(result1 is BridgeParse.Rejected)

        val result2 = BridgeCodec.parse("   ", 1000L)
        assertTrue(result2 is BridgeParse.Rejected)
    }

    @Test
    fun parse_plainTextNotice_returnsNotice() {
        val line = "[BOOT] ESP32 Rover Firmware v1.2 Initialized"
        val result = BridgeCodec.parse(line, 1000L)

        assertTrue(result is BridgeParse.Notice)
        assertEquals(line, (result as BridgeParse.Notice).text)
    }

    @Test
    fun encodeCommand_producesValidJson() {
        val encoded = BridgeCodec.encodeCommand("SPEED", 0.75f)
        assertTrue(encoded.contains("\"command\":\"SPEED\"") || encoded.contains("\"command\": \"SPEED\""))
        assertTrue(encoded.contains("0.75"))
    }
}
