package com.minesafety.roboeye.camera

import android.hardware.camera2.CameraCharacteristics
import org.junit.Assert.assertEquals
import org.junit.Test

class Camera2CapabilityHelperTest {

    @Test
    fun hardwareLevelToString_mapsCorrectly() {
        assertEquals("LEGACY", Camera2CapabilityHelper.hardwareLevelToString(CameraCharacteristics.INFO_SUPPORTED_HARDWARE_LEVEL_LEGACY))
        assertEquals("LIMITED", Camera2CapabilityHelper.hardwareLevelToString(CameraCharacteristics.INFO_SUPPORTED_HARDWARE_LEVEL_LIMITED))
        assertEquals("FULL", Camera2CapabilityHelper.hardwareLevelToString(CameraCharacteristics.INFO_SUPPORTED_HARDWARE_LEVEL_FULL))
        assertEquals("LEVEL_3", Camera2CapabilityHelper.hardwareLevelToString(CameraCharacteristics.INFO_SUPPORTED_HARDWARE_LEVEL_3))
        assertEquals("EXTERNAL", Camera2CapabilityHelper.hardwareLevelToString(4))
    }
}
