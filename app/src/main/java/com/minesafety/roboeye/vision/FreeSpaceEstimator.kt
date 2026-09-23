package com.minesafety.roboeye.vision

import com.minesafety.roboeye.core.model.CameraFrame
import com.minesafety.roboeye.core.model.FreeSpaceObservation

/**
 * Geometric free-space estimator boundary interface (drivable ground corridor).
 */
interface FreeSpaceEstimator {
    fun estimateFreeSpace(frame: CameraFrame, cameraPitchDeg: Float): FreeSpaceObservation
}
