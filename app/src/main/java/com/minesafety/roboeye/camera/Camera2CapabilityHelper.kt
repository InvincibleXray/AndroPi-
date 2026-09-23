package com.minesafety.roboeye.camera

import android.content.Context
import android.graphics.ImageFormat
import android.hardware.camera2.CameraCharacteristics
import android.hardware.camera2.CameraManager
import android.util.Size

/**
 * Probes device Camera2 capabilities to determine if Camera2 is safely supported
 * without falling back into slow, broken, or emulated LEGACY HAL drivers.
 */
object Camera2CapabilityHelper {

    data class CapabilityResult(
        val isSupported: Boolean,
        val cameraId: String? = null,
        val hardwareLevel: Int = -1,
        val hardwareLevelName: String = "UNKNOWN",
        val supportedSizes: List<Size> = emptyList(),
        val reason: String = "",
    )

    /**
     * Probes the default back-facing camera for Camera2 support.
     * Rejects devices with LEGACY hardware level or missing YUV_420_888 output streams.
     */
    fun probeCamera2(context: Context, targetWidth: Int = 640, targetHeight: Int = 360): CapabilityResult {
        val cameraManager = context.getSystemService(Context.CAMERA_SERVICE) as? CameraManager
            ?: return CapabilityResult(
                isSupported = false,
                reason = "CameraManager system service unavailable"
            )

        return probeCamera2(cameraManager, targetWidth, targetHeight)
    }

    /**
     * Probes using an explicit CameraManager instance (allows unit testing / injection).
     */
    fun probeCamera2(cameraManager: CameraManager, targetWidth: Int = 640, targetHeight: Int = 360): CapabilityResult {
        try {
            val cameraIds = cameraManager.cameraIdList
            if (cameraIds.isEmpty()) {
                return CapabilityResult(isSupported = false, reason = "No camera IDs found on device")
            }

            var chosenId: String? = null
            var chosenCharacteristics: CameraCharacteristics? = null

            for (id in cameraIds) {
                val chars = cameraManager.getCameraCharacteristics(id)
                val facing = chars.get(CameraCharacteristics.LENS_FACING)
                if (facing == CameraCharacteristics.LENS_FACING_BACK) {
                    chosenId = id
                    chosenCharacteristics = chars
                    break
                }
            }

            // Fallback to first camera if no back-facing found
            if (chosenId == null) {
                chosenId = cameraIds[0]
                chosenCharacteristics = cameraManager.getCameraCharacteristics(chosenId)
            }

            val chars = chosenCharacteristics ?: return CapabilityResult(isSupported = false, reason = "Failed to load camera characteristics")
            val level = chars.get(CameraCharacteristics.INFO_SUPPORTED_HARDWARE_LEVEL) ?: -1
            val levelName = hardwareLevelToString(level)

            // Reject LEGACY HAL: emulated camera1 bridge with high latency and broken YUV planes
            if (level == CameraCharacteristics.INFO_SUPPORTED_HARDWARE_LEVEL_LEGACY) {
                return CapabilityResult(
                    isSupported = false,
                    cameraId = chosenId,
                    hardwareLevel = level,
                    hardwareLevelName = levelName,
                    reason = "Device only supports Camera2 LEGACY HAL; CameraX fallback required"
                )
            }

            val map = chars.get(CameraCharacteristics.SCALER_STREAM_CONFIGURATION_MAP)
            if (map == null) {
                return CapabilityResult(
                    isSupported = false,
                    cameraId = chosenId,
                    hardwareLevel = level,
                    hardwareLevelName = levelName,
                    reason = "StreamConfigurationMap unavailable"
                )
            }

            val yuvSizes = map.getOutputSizes(ImageFormat.YUV_420_888)?.toList() ?: emptyList()
            if (yuvSizes.isEmpty()) {
                return CapabilityResult(
                    isSupported = false,
                    cameraId = chosenId,
                    hardwareLevel = level,
                    hardwareLevelName = levelName,
                    reason = "No YUV_420_888 output sizes supported"
                )
            }

            return CapabilityResult(
                isSupported = true,
                cameraId = chosenId,
                hardwareLevel = level,
                hardwareLevelName = levelName,
                supportedSizes = yuvSizes,
                reason = "Camera2 fully supported with $levelName HAL"
            )
        } catch (e: Exception) {
            return CapabilityResult(
                isSupported = false,
                reason = "Capability probe exception: ${e.message ?: e::class.java.simpleName}"
            )
        }
    }

    fun hardwareLevelToString(level: Int): String = when (level) {
        CameraCharacteristics.INFO_SUPPORTED_HARDWARE_LEVEL_LEGACY -> "LEGACY"
        CameraCharacteristics.INFO_SUPPORTED_HARDWARE_LEVEL_LIMITED -> "LIMITED"
        CameraCharacteristics.INFO_SUPPORTED_HARDWARE_LEVEL_FULL -> "FULL"
        CameraCharacteristics.INFO_SUPPORTED_HARDWARE_LEVEL_3 -> "LEVEL_3"
        4 -> "EXTERNAL" // INFO_SUPPORTED_HARDWARE_LEVEL_EXTERNAL (API 28+)
        else -> "UNKNOWN ($level)"
    }
}
