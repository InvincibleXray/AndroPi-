package com.minesafety.roboeye.nav

import com.minesafety.roboeye.core.model.MotionCommand
import com.minesafety.roboeye.core.model.SafetyState
import com.minesafety.roboeye.localization.TrackingQuality
import com.minesafety.roboeye.nav.arbiter.ArbitratedCommand
import com.minesafety.roboeye.nav.arbiter.CommandArbiter
import com.minesafety.roboeye.nav.avoidance.AvoidanceAction
import com.minesafety.roboeye.nav.avoidance.LocalObstacleAvoidance
import com.minesafety.roboeye.nav.control.DifferentialDriveController
import com.minesafety.roboeye.nav.control.MotionIntent
import com.minesafety.roboeye.nav.model.GoalValidationResult
import com.minesafety.roboeye.nav.model.GoalValidator
import com.minesafety.roboeye.nav.model.LocalGoal
import com.minesafety.roboeye.nav.model.NavigationWorldModel
import com.minesafety.roboeye.nav.planner.AStarGridPlanner
import com.minesafety.roboeye.nav.planner.PathValidator
import com.minesafety.roboeye.nav.planner.PlannedPath
import com.minesafety.roboeye.nav.planner.PlanningResult
import com.minesafety.roboeye.nav.safety.NavigationSafetyGate
import com.minesafety.roboeye.nav.safety.SafetyGateAction
import com.minesafety.roboeye.nav.safety.SafetyGateVerdict
import com.minesafety.roboeye.nav.state.AutonomousNavigationStateMachine
import com.minesafety.roboeye.nav.state.NavigationState
import com.minesafety.roboeye.nav.transport.RoverCommandSender
import com.minesafety.roboeye.vision.GeometryTrustLevel
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlin.math.hypot

/**
 * Master Autonomous Navigation Orchestrator.
 *
 * Implements the deterministic closed-loop pipeline:
 * GOAL -> WORLD MODEL -> LOCALIZATION -> NAVIGATOR (A*) -> LOCAL AVOIDANCE ->
 * MOTION CONTROLLER -> COMMAND ARBITER -> DETERMINISTIC SAFETY GATE -> ESP32 SENDER.
 */
class AutonomousNavigator(
    val planner: AStarGridPlanner = AStarGridPlanner(),
    val pathValidator: PathValidator = PathValidator(),
    val avoidance: LocalObstacleAvoidance = LocalObstacleAvoidance(pathValidator),
    val controller: DifferentialDriveController = DifferentialDriveController(),
    val arbiter: CommandArbiter = CommandArbiter(),
    val safetyGate: NavigationSafetyGate = NavigationSafetyGate(),
    val commandSender: RoverCommandSender,
    val stateMachine: AutonomousNavigationStateMachine = AutonomousNavigationStateMachine(),
) {
    private val _activeGoal = MutableStateFlow<LocalGoal?>(null)
    val activeGoal: StateFlow<LocalGoal?> = _activeGoal.asStateFlow()

    private val _activePath = MutableStateFlow<PlannedPath?>(null)
    val activePath: StateFlow<PlannedPath?> = _activePath.asStateFlow()

    private val _lastArbitratedCommand = MutableStateFlow<ArbitratedCommand?>(null)
    val lastArbitratedCommand: StateFlow<ArbitratedCommand?> = _lastArbitratedCommand.asStateFlow()

    private val _lastSafetyVerdict = MutableStateFlow<SafetyGateVerdict?>(null)
    val lastSafetyVerdict: StateFlow<SafetyGateVerdict?> = _lastSafetyVerdict.asStateFlow()

    private var manualEmergencyStop = false
    private var manualCommand: MotionCommand? = null

    /**
     * Submits a new local navigation goal, validating it against world bounds and occupancy.
     */
    fun setGoal(goal: LocalGoal, worldModel: NavigationWorldModel): GoalValidationResult {
        val validation = GoalValidator.validate(goal, worldModel)
        if (!validation.isValid) {
            return validation
        }

        _activeGoal.value = goal
        // Plan initial path
        when (val planResult = planner.planPath(worldModel, goal)) {
            is PlanningResult.Success -> {
                _activePath.value = planResult.path
                stateMachine.transition(NavigationState.READY, "Valid path planned (${planResult.path.size} waypoints)")
            }
            is PlanningResult.Failure -> {
                _activePath.value = null
                stateMachine.transition(NavigationState.STOPPING, "Initial planning failed: ${planResult.reason}")
            }
        }

        return GoalValidationResult(true)
    }

    fun clearGoal() {
        _activeGoal.value = null
        _activePath.value = null
        if (stateMachine.currentState == NavigationState.NAVIGATING ||
            stateMachine.currentState == NavigationState.AVOIDING ||
            stateMachine.currentState == NavigationState.SLOWING
        ) {
            stateMachine.transition(NavigationState.IDLE, "Goal cleared by operator")
        }
    }

    fun triggerEmergencyStop(reason: String = "Operator Emergency Stop") {
        manualEmergencyStop = true
        stateMachine.transition(NavigationState.ESTOP, reason)
    }

    fun clearEmergencyStop() {
        manualEmergencyStop = false
        stateMachine.reset()
    }

    fun setManualCommand(command: MotionCommand?) {
        manualCommand = command
    }

    /**
     * Primary deterministic navigation tick running at 5-10 Hz.
     */
    suspend fun tick(
        worldModel: NavigationWorldModel,
        safetyState: SafetyState,
        isTransportConnected: Boolean,
        nowMs: Long = System.currentTimeMillis(),
    ): ArbitratedCommand {
        // 1. High-Priority Platform Safety Checks
        if (manualEmergencyStop || safetyState.isEmergencyStop) {
            stateMachine.transition(NavigationState.ESTOP, safetyState.reasons.firstOrNull() ?: "E-Stop active", nowMs)
        } else if (!isTransportConnected) {
            stateMachine.transition(NavigationState.LINK_LOST, "ESP32 transport disconnected", nowMs)
        } else if (worldModel.pose.trackingState == TrackingQuality.LOST) {
            stateMachine.transition(NavigationState.LOCALIZATION_LOST, "Visual-inertial tracking lost", nowMs)
        } else if (worldModel.geometryTrust == GeometryTrustLevel.UNTRUSTED) {
            stateMachine.transition(NavigationState.PERCEPTION_UNCERTAIN, "Perception geometry untrusted", nowMs)
        }

        val currentGoal = _activeGoal.value
        val currentPath = _activePath.value
        var autonomousProposed = MotionCommand.STOP

        // 2. Autonomous Navigation Processing
        if (currentGoal != null && currentGoal.valid && !manualEmergencyStop && isTransportConnected && worldModel.isNavigable) {
            val distToGoal = hypot(currentGoal.targetX - worldModel.pose.xM, currentGoal.targetY - worldModel.pose.yM)

            // Check if arrived at goal
            if (distToGoal <= currentGoal.positionToleranceM) {
                stateMachine.transition(NavigationState.GOAL_REACHED, "Reached goal tolerance zone ($distToGoal m)", nowMs)
                _activeGoal.value = null
                _activePath.value = null
                autonomousProposed = MotionCommand.STOP.copy(source = "GOAL_REACHED")
            } else {
                // Tactical Avoidance & Corridor Assessment
                val avoidanceAction = avoidance.evaluate(worldModel, currentPath, nowMs)
                var effectiveSpeedLimit = controller.maxLinearVelocityMps

                when (avoidanceAction) {
                    AvoidanceAction.Clear -> {
                        stateMachine.transition(NavigationState.NAVIGATING, "Path clear", nowMs)
                    }
                    is AvoidanceAction.SlowDown -> {
                        stateMachine.transition(NavigationState.SLOWING, avoidanceAction.reason, nowMs)
                        effectiveSpeedLimit = avoidanceAction.speedLimitMps
                    }
                    is AvoidanceAction.Stop -> {
                        stateMachine.transition(NavigationState.STOPPING, avoidanceAction.reason, nowMs)
                        effectiveSpeedLimit = 0.0f
                    }
                    is AvoidanceAction.EmergencyBrake -> {
                        stateMachine.transition(NavigationState.STOPPING, avoidanceAction.reason, nowMs)
                        effectiveSpeedLimit = 0.0f
                    }
                    is AvoidanceAction.ReplanRequired -> {
                        stateMachine.transition(NavigationState.REPLANNING, avoidanceAction.reason, nowMs)
                        // Trigger dynamic replan
                        when (val replan = planner.planPath(worldModel, currentGoal)) {
                            is PlanningResult.Success -> {
                                _activePath.value = replan.path
                                stateMachine.transition(NavigationState.NAVIGATING, "Replanned around obstacle", nowMs)
                            }
                            is PlanningResult.Failure -> {
                                _activePath.value = null
                                stateMachine.transition(NavigationState.STOPPING, "Replan failed: ${replan.reason}", nowMs)
                                effectiveSpeedLimit = 0.0f
                            }
                        }
                    }
                }

                // Compute motion intent if path is available and speed > 0
                val activeOrReplannedPath = _activePath.value
                if (activeOrReplannedPath != null && !activeOrReplannedPath.isEmpty && effectiveSpeedLimit > 0.0f) {
                    val intent = controller.computeMotion(
                        path = activeOrReplannedPath,
                        currentPose = worldModel.pose,
                        goal = currentGoal,
                        speedLimitMps = effectiveSpeedLimit,
                        nowMs = nowMs,
                    )
                    autonomousProposed = intent.toMotionCommand()
                } else {
                    autonomousProposed = MotionCommand.STOP.copy(source = "STOPPED_${stateMachine.currentState}")
                }
            }
        } else if (currentGoal == null && stateMachine.currentState == NavigationState.READY) {
            stateMachine.transition(NavigationState.IDLE, "No active goal", nowMs)
        }

        // 3. Command Arbitration (Deterministic Priority Hierarchy)
        val arbitrated = arbiter.arbitrate(
            hardwareEStop = false,
            safetyState = safetyState,
            isTransportConnected = isTransportConnected,
            localizationQuality = worldModel.pose.trackingState,
            manualEmergencyStop = manualEmergencyStop,
            manualCommand = manualCommand,
            autonomousCommand = if (currentGoal != null) autonomousProposed else null,
            nowMs = nowMs,
        )
        _lastArbitratedCommand.value = arbitrated

        // 4. Deterministic Safety Gate
        val verdict = safetyGate.vetCommand(
            proposed = arbitrated.command,
            worldModel = worldModel,
            safetyState = safetyState,
            isTransportConnected = isTransportConnected,
            nowMs = nowMs,
        )
        _lastSafetyVerdict.value = verdict

        // 5. Transmit Safe Command to ESP32
        commandSender.sendCommand(verdict.safeCommand, nowMs)
        commandSender.maybeSendHeartbeat(nowMs)

        return arbitrated
    }
}
