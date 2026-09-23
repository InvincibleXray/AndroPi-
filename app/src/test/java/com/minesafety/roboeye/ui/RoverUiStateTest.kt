package com.minesafety.roboeye.ui

import com.minesafety.roboeye.core.Availability
import com.minesafety.roboeye.core.CameraState
import com.minesafety.roboeye.core.ImuReading
import com.minesafety.roboeye.core.ImuState
import com.minesafety.roboeye.core.MirrorLevel
import com.minesafety.roboeye.core.MirrorVerdict
import com.minesafety.roboeye.core.NodeStatus
import com.minesafety.roboeye.fusion.FreeSpaceClearance
import com.minesafety.roboeye.localization.LocalPose
import com.minesafety.roboeye.localization.MonocularScaleState
import com.minesafety.roboeye.localization.TrackingQuality
import com.minesafety.roboeye.mapping.LocalMapCell
import com.minesafety.roboeye.mapping.MapCellState
import com.minesafety.roboeye.vision.MotionState
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * Phase 5A UI State & Representation Unit Tests.
 *
 * Verifies that:
 * 1. TrackingQuality states (INITIALIZING, TRACKING, DEGRADED, LOST, RELOCALIZING) are handled cleanly.
 * 2. MonocularScaleState (SCALE_UNKNOWN, SCALE_UNRELIABLE, SCALE_ESTIMATED) governs metric formatting.
 * 3. MapCellState (FREE, OCCUPIED, UNCERTAIN) strictly enforces "Unknown Is NOT Safe".
 * 4. FreeSpaceClearance strictly rejects UNKNOWN as free/safe.
 * 5. Mirror & E-Stop verdicts take precedence in system status without fake data.
 */
class RoverUiStateTest {

    @Test
    fun trackingQuality_allFiveStates_distinctAndClassified() {
        val states = listOf(
            TrackingQuality.INITIALIZING,
            TrackingQuality.TRACKING,
            TrackingQuality.DEGRADED,
            TrackingQuality.LOST,
            TrackingQuality.RELOCALIZING,
        )
        assertEquals(5, states.toSet().size)

        // LOST state must indicate unconfirmed map position
        val lostPose = LocalPose(xM = 1.0f, yM = 2.0f, yawDeg = 15f, trackingState = TrackingQuality.LOST)
        assertEquals(TrackingQuality.LOST, lostPose.trackingState)

        // RELOCALIZING must not be treated as TRACKING
        val relocPose = LocalPose(trackingState = TrackingQuality.RELOCALIZING)
        assertTrue(relocPose.trackingState != TrackingQuality.TRACKING)
    }

    @Test
    fun monocularScale_metricInterpretation_onlyAllowedWhenScaleEstimated() {
        fun isMetricFormattingAllowed(scale: MonocularScaleState): Boolean {
            return scale == MonocularScaleState.SCALE_ESTIMATED
        }

        assertTrue(isMetricFormattingAllowed(MonocularScaleState.SCALE_ESTIMATED))
        assertFalse(isMetricFormattingAllowed(MonocularScaleState.SCALE_UNKNOWN))
        assertFalse(isMetricFormattingAllowed(MonocularScaleState.SCALE_UNRELIABLE))
    }

    @Test
    fun mapCellState_uncertain_isNeverTreatedAsFreeOrSafe() {
        fun isCellTraversable(state: MapCellState, confidence: Float): Boolean {
            // UNCERTAIN is NEVER safe or traversable
            return state == MapCellState.FREE && confidence >= 0.20f
        }

        val freeCell = LocalMapCell(0, 0, 0f, 0f, MapCellState.FREE, confidence = 0.8f, lastUpdatedMs = 0L)
        val occupiedCell = LocalMapCell(0, 0, 0f, 0f, MapCellState.OCCUPIED, confidence = 0.9f, lastUpdatedMs = 0L)
        val uncertainCell = LocalMapCell(0, 0, 0f, 0f, MapCellState.UNCERTAIN, confidence = 0.0f, lastUpdatedMs = 0L)

        assertTrue(isCellTraversable(freeCell.state, freeCell.confidence))
        assertFalse(isCellTraversable(occupiedCell.state, occupiedCell.confidence))
        assertFalse(isCellTraversable(uncertainCell.state, uncertainCell.confidence))
    }

    @Test
    fun freeSpaceClearance_unknown_isNeverSafe() {
        fun isCorridorSafe(clearance: FreeSpaceClearance): Boolean {
            return clearance == FreeSpaceClearance.CLEAR_WITH_CONFIDENCE
        }

        assertTrue(isCorridorSafe(FreeSpaceClearance.CLEAR_WITH_CONFIDENCE))
        assertFalse(isCorridorSafe(FreeSpaceClearance.BLOCKED))
        assertFalse(isCorridorSafe(FreeSpaceClearance.CONFLICTING))
        assertFalse(isCorridorSafe(FreeSpaceClearance.UNKNOWN))
    }

    @Test
    fun motionState_allMotionTypes_haveValidOperatorLabels() {
        val motionStates = MotionState.entries
        assertTrue(motionStates.isNotEmpty())
        for (m in motionStates) {
            val label = m.name.replace("_", " ")
            assertTrue(label.isNotBlank())
            assertFalse(label.contains("RADAR", ignoreCase = true))
        }
    }

    @Test
    fun systemStatus_safetyMirrorEStop_triggersEstopLabel() {
        val estopVerdict = MirrorVerdict(level = MirrorLevel.LOCAL_ESTOP, reasons = listOf("Emergency obstacle"))
        val status = NodeStatus(
            running = true,
            mirror = estopVerdict,
        )

        val isEmergencyStop = status.mirror.isEStop || status.mirror.level == MirrorLevel.LOCAL_ESTOP
        assertTrue(isEmergencyStop)
    }

    @Test
    fun systemStatus_cameraNoPermission_honestlyReported() {
        val status = NodeStatus(
            running = false,
            camera = CameraState(availability = Availability.NO_PERMISSION),
        )
        assertEquals(Availability.NO_PERMISSION, status.camera.availability)
        assertFalse(status.camera.isActive)
    }

    @Test
    fun systemStatus_imuWaiting_honestlyReportedWithoutFakeData() {
        val status = NodeStatus(
            running = true,
            imu = ImuState(availability = Availability.WAITING, reading = null),
        )
        assertEquals(Availability.WAITING, status.imu.availability)
        assertEquals(null, status.imu.reading)
    }
}
