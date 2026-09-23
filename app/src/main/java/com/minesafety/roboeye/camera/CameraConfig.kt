package com.minesafety.roboeye.camera

/**
 * Camera operational configuration.
 */
data class CameraConfig(
    val targetWidth: Int = 640,
    val targetHeight: Int = 360,
    val targetFps: Float = 10.0f,
    val jpegQuality: Int = 65,
    /** When false, camera analyzer executes without binding or allocating UI preview surfaces. */
    val enablePreview: Boolean = false,
    /** Whether to attempt the optional Camera2 API (default: false, CameraX active). */
    val useCamera2: Boolean = false,
) {
    val intervalMs: Long
        get() = (1000.0f / targetFps.coerceIn(1.0f, 60.0f)).toLong()

    companion object {
        val LOW_END_DEFAULT = CameraConfig(
            targetWidth = 640,
            targetHeight = 360,
            targetFps = 10.0f,
            enablePreview = false,
            useCamera2 = false,
        )
    }
}
