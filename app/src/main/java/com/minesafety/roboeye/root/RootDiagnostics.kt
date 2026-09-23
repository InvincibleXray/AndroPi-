package com.minesafety.roboeye.root

import java.io.File

/**
 * Non-invasive capability boundary for optional advanced root diagnostics.
 *
 * Guaranteed Safety:
 * - The application operates 100% normally WITHOUT root.
 * - Does NOT require root permissions.
 * - Does NOT execute privileged su commands or modify system partitions.
 */
object RootDiagnostics {

    private val suPaths = arrayOf(
        "/system/app/Superuser.apk",
        "/sbin/su",
        "/system/bin/su",
        "/system/xbin/su",
        "/data/local/xbin/su",
        "/data/local/bin/su",
        "/system/sd/xbin/su",
        "/system/bin/failsafe/su",
        "/data/local/su",
    )

    /**
     * Checks if the device appears to have a su binary available.
     * Safe, non-invasive check (file existence only; never invokes su).
     */
    val isRootAvailable: Boolean by lazy {
        try {
            suPaths.any { path -> File(path).exists() }
        } catch (_: Throwable) {
            false
        }
    }

    /** Human-readable system execution mode label. */
    val executionModeLabel: String
        get() = if (isRootAvailable) "Advanced Mode (Root Available)" else "Standard Android Mode (Unrooted)"
}
