package com.minesafety.roboeye.core.config

/**
 * Performance profile tailored to the device hardware class.
 *
 * Defaults to LOW_END to ensure smooth, thermal-safe operation on Redmi 6A-class hardware.
 */
enum class PerformanceProfile(
    val maxCameraFps: Float,
    val cameraWidth: Int,
    val cameraHeight: Int,
    val enableCameraPreviewByDefault: Boolean,
    val maxVisionHz: Float,
    val enableYoloByDefault: Boolean,
    val bufferPoolSize: Int,
) {
    /** MediaTek Helio A22 / 2 GB RAM (Redmi 6A). Minimal allocations, 360p resolution, headless preview, no YOLO. */
    LOW_END(
        maxCameraFps = 10.0f,
        cameraWidth = 640,
        cameraHeight = 360,
        enableCameraPreviewByDefault = false,
        maxVisionHz = 10.0f,
        enableYoloByDefault = false,
        bufferPoolSize = 2,
    ),

    /** Mid-range hardware (Snapdragon 600/700 series, 4-6 GB RAM). */
    BALANCED(
        maxCameraFps = 15.0f,
        cameraWidth = 1280,
        cameraHeight = 720,
        enableCameraPreviewByDefault = true,
        maxVisionHz = 12.0f,
        enableYoloByDefault = false,
        bufferPoolSize = 4,
    ),

    /** High-tier hardware (Snapdragon 8 Gen / Dimensity 9000+, 8+ GB RAM). */
    HIGH_PERFORMANCE(
        maxCameraFps = 30.0f,
        cameraWidth = 1920,
        cameraHeight = 1080,
        enableCameraPreviewByDefault = true,
        maxVisionHz = 20.0f,
        enableYoloByDefault = true,
        bufferPoolSize = 6,
    ),
}

/**
 * Central configuration for Rover Brain runtime parameters.
 */
data class RoverBrainConfig(
    val profile: PerformanceProfile = PerformanceProfile.LOW_END,
    val vehicleId: String = "ROVER-01",
    val maxTiltDeg: Float = 25.0f,
    val eStopDistanceM: Float = 0.5f,
    val slowDownDistanceM: Float = 1.5f,
    val minVisibilityScore: Float = 0.10f,
    val heartbeatTimeoutMs: Long = 1_000L,
    val cameraJpegQuality: Int = 65,
    val isAutonomousHeadless: Boolean = true,
    val useCamera2: Boolean = false,
    val maxFeatureCount: Int = 150,
    val visionFps: Float = 10.0f,
)
