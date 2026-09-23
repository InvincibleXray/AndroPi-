package com.minesafety.roboeye.nav

import com.minesafety.roboeye.core.model.MotionCommand
import com.minesafety.roboeye.core.model.SafetyLevel
import com.minesafety.roboeye.core.model.SafetyState
import com.minesafety.roboeye.localization.LocalPose
import com.minesafety.roboeye.localization.TrackingQuality
import com.minesafety.roboeye.mapping.LocalSpatialMap
import com.minesafety.roboeye.nav.arbiter.CommandArbiter
import com.minesafety.roboeye.nav.arbiter.CommandPriority
import com.minesafety.roboeye.nav.model.NavigationWorldModel
import com.minesafety.roboeye.nav.safety.NavigationSafetyGate
import com.minesafety.roboeye.nav.safety.SafetyGateAction
import com.minesafety.roboeye.vision.GeometryTrustLevel
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Before
import org.junit.Test

/**
 * Deterministic unit tests covering:
 * M. Command arbitration (strict 8-tier hierarchy)
 * N. Safety override (safety gate overrules forward planner intent)
 * O. E-stop priority (highest priority over all inputs)
 * P. Localization lost (autonomous motion halts)
 * Q. Perception uncertain (untrusted geometry halts rover)
 * R. Map stale (stale observations trigger safety stop)
 * S. Link lost (transport disconnect triggers safe stop)
 * X. Manual mode regression (teleoperation commands correctly clamped by safety gate)
 */
class CommandArbitrationAndSafetyTest {

    private lateinit var arbiter: CommandArbiter
    private lateinit var safetyGate: NavigationSafetyGate
    private lateinit var emptyMap: LocalSpatialMap

    @Before
    fun setUp() {
        arbiter = CommandArbiter()
        safetyGate = NavigationSafetyGate()
        emptyMap = LocalSpatialMap()
    }

    private fun createWorldModel(
        pose: LocalPose = LocalPose.ORIGIN.copy(trackingState = TrackingQuality.TRACKING),
        geometryTrust: GeometryTrustLevel = GeometryTrustLevel.TRUSTED,
        timestampMs: Long = System.currentTimeMillis(),
    ): NavigationWorldModel {
        return NavigationWorldModel(
            pose = pose,
            spatialMap = emptyMap,
            geometryTrust = geometryTrust,
            timestampMs = timestampMs,
        )
    }

    // --- M. COMMAND ARBITRATION HIERARCHY TESTS ---
    @Test
    fun testCommandArbiter_EnforcesHierarchy_HardwareEStopOverAutonomousMotion() {
        val autoCmd = MotionCommand(linearVelocityMps = 0.35f, source = "AUTONOMOUS_NAV")
        val arbitrated = arbiter.arbitrate(
            hardwareEStop = true, // Priority 1
            safetyState = SafetyState.NOMINAL,
            isTransportConnected = true,
            localizationQuality = TrackingQuality.TRACKING,
            manualEmergencyStop = false,
            autonomousCommand = autoCmd,
        )

        assertEquals(CommandPriority.PRIORITY_1_HARDWARE_ESTOP, arbitrated.activePriority)
        assertTrue(arbitrated.command.emergencyStop)
        assertEquals(0.0f, arbitrated.command.linearVelocityMps, 0.001f)
    }

    @Test
    fun testCommandArbiter_EnforcesHierarchy_SafetyFaultOverAutonomousMotion() {
        val autoCmd = MotionCommand(linearVelocityMps = 0.35f, source = "AUTONOMOUS_NAV")
        val safetyFault = SafetyState(
            level = SafetyLevel.EMERGENCY_STOP,
            isEmergencyStop = true,
            reasons = listOf("Critical tilt hazard: 28° > 25°"),
        )

        val arbitrated = arbiter.arbitrate(
            hardwareEStop = false,
            safetyState = safetyFault, // Priority 2
            isTransportConnected = true,
            localizationQuality = TrackingQuality.TRACKING,
            manualEmergencyStop = false,
            autonomousCommand = autoCmd,
        )

        assertEquals(CommandPriority.PRIORITY_2_SAFETY_FAULT, arbitrated.activePriority)
        assertTrue(arbitrated.command.emergencyStop)
    }

    // --- O. E-STOP PRIORITY TESTS ---
    @Test
    fun testEStopPriority_OverridesAllSubsystems() {
        val autoCmd = MotionCommand(linearVelocityMps = 0.35f, source = "AUTONOMOUS_NAV")
        val manualCmd = MotionCommand(linearVelocityMps = 0.50f, source = "MANUAL_TELEOP")

        val arbitrated = arbiter.arbitrate(
            hardwareEStop = false,
            safetyState = SafetyState.NOMINAL,
            isTransportConnected = true,
            localizationQuality = TrackingQuality.TRACKING,
            manualEmergencyStop = true, // Priority 5
            manualCommand = manualCmd,
            autonomousCommand = autoCmd,
        )

        assertEquals(CommandPriority.PRIORITY_5_MANUAL_EMERGENCY_OVERRIDE, arbitrated.activePriority)
        assertTrue(arbitrated.command.emergencyStop)
    }

    // --- P. LOCALIZATION LOST TESTS ---
    @Test
    fun testCommandArbiter_LocalizationLostForcesHalt() {
        val autoCmd = MotionCommand(linearVelocityMps = 0.35f, source = "AUTONOMOUS_NAV")

        val arbitrated = arbiter.arbitrate(
            hardwareEStop = false,
            safetyState = SafetyState.NOMINAL,
            isTransportConnected = true,
            localizationQuality = TrackingQuality.LOST, // Priority 4
            manualEmergencyStop = false,
            autonomousCommand = autoCmd,
        )

        assertEquals(CommandPriority.PRIORITY_4_LOCALIZATION_FAILURE, arbitrated.activePriority)
        assertEquals(0.0f, arbitrated.command.linearVelocityMps, 0.001f)
        assertEquals(0.0f, arbitrated.command.angularVelocityRadS, 0.001f)
    }

    // --- S. LINK LOST TESTS ---
    @Test
    fun testCommandArbiter_LinkLostForcesHalt() {
        val autoCmd = MotionCommand(linearVelocityMps = 0.35f, source = "AUTONOMOUS_NAV")

        val arbitrated = arbiter.arbitrate(
            hardwareEStop = false,
            safetyState = SafetyState.NOMINAL,
            isTransportConnected = false, // Priority 3
            localizationQuality = TrackingQuality.TRACKING,
            manualEmergencyStop = false,
            autonomousCommand = autoCmd,
        )

        assertEquals(CommandPriority.PRIORITY_3_COMMUNICATION_FAILURE, arbitrated.activePriority)
        assertEquals(0.0f, arbitrated.command.linearVelocityMps, 0.001f)
    }

    // --- N. SAFETY OVERRIDE (DETERMINISTIC SAFETY GATE) TESTS ---
    @Test
    fun testSafetyGate_OverridesProposedMotionWhenEStopTriggered() {
        val proposed = MotionCommand(linearVelocityMps = 0.35f, source = "AUTONOMOUS_NAV")
        val worldModel = createWorldModel()
        val safetyState = SafetyState(
            level = SafetyLevel.EMERGENCY_STOP,
            isEmergencyStop = true,
            reasons = listOf("Obstacle collision danger"),
        )

        val verdict = safetyGate.vetCommand(proposed, worldModel, safetyState, isTransportConnected = true)

        assertEquals(SafetyGateAction.ESTOP, verdict.action)
        assertTrue(verdict.safeCommand.emergencyStop)
        assertEquals(0.0f, verdict.safeCommand.linearVelocityMps, 0.001f)
    }

    // --- Q. PERCEPTION UNCERTAIN TESTS ---
    @Test
    fun testSafetyGate_UntrustedPerceptionForcesSafeStop() {
        val proposed = MotionCommand(linearVelocityMps = 0.35f, source = "AUTONOMOUS_NAV")
        val worldModel = createWorldModel(geometryTrust = GeometryTrustLevel.UNTRUSTED)

        val verdict = safetyGate.vetCommand(proposed, worldModel, SafetyState.NOMINAL, isTransportConnected = true)

        assertEquals(SafetyGateAction.STOP, verdict.action)
        assertEquals(0.0f, verdict.safeCommand.linearVelocityMps, 0.001f)
        assertTrue(verdict.reasons.any { it.contains("geometry", ignoreCase = true) })
    }

    // --- R. MAP STALE TESTS ---
    @Test
    fun testSafetyGate_StaleWorldModelForcesSafeStop() {
        val proposed = MotionCommand(linearVelocityMps = 0.35f, source = "AUTONOMOUS_NAV")
        val now = System.currentTimeMillis()
        // World model is 2500ms old (> 1000ms max age)
        val staleModel = createWorldModel(timestampMs = now - 2500L)

        val verdict = safetyGate.vetCommand(proposed, staleModel, SafetyState.NOMINAL, isTransportConnected = true, nowMs = now)

        assertEquals(SafetyGateAction.STOP, verdict.action)
        assertEquals(0.0f, verdict.safeCommand.linearVelocityMps, 0.001f)
        assertTrue(verdict.reasons.any { it.contains("stale", ignoreCase = true) })
    }

    // --- X. MANUAL MODE REGRESSION TESTS ---
    @Test
    fun testManualTeleop_ClampedBySafetyGateCaution() {
        val proposedManual = MotionCommand(linearVelocityMps = 0.60f, source = "MANUAL_TELEOP")
        val worldModel = createWorldModel()
        // Safety state is CAUTION with speed limit 0.20 m/s
        val cautionSafety = SafetyState(
            level = SafetyLevel.CAUTION,
            speedLimitMps = 0.20f,
            reasons = listOf("Obstacle caution zone"),
        )

        val verdict = safetyGate.vetCommand(proposedManual, worldModel, cautionSafety, isTransportConnected = true)

        assertEquals(SafetyGateAction.SLOW, verdict.action)
        assertEquals(0.20f, verdict.safeCommand.linearVelocityMps, 0.001f)
    }
}
