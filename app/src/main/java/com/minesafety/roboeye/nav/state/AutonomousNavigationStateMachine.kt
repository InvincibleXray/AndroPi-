package com.minesafety.roboeye.nav.state

import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow

/**
 * Deterministic operational states of the autonomous navigation architecture.
 */
enum class NavigationState(val label: String) {
    IDLE("IDLE"),
    INITIALIZING("INITIALIZING"),
    LOCALIZING("LOCALIZING"),
    READY("READY"),
    NAVIGATING("NAVIGATING"),
    SLOWING("SLOWING"),
    AVOIDING("AVOIDING"),
    STOPPING("STOPPING"),
    GOAL_REACHED("GOAL REACHED"),
    REPLANNING("REPLANNING"),
    LOCALIZATION_LOST("LOCALIZATION LOST"),
    PERCEPTION_UNCERTAIN("PERCEPTION UNCERTAIN"),
    LINK_LOST("LINK LOST"),
    ESTOP("EMERGENCY STOP"),
    FAULT("SYSTEM FAULT"),
}

/**
 * Immutable snapshot of navigation state with transition reasoning.
 */
data class NavigationStateSnapshot(
    val state: NavigationState = NavigationState.IDLE,
    val reason: String = "System initialized",
    val timestampMs: Long = System.currentTimeMillis(),
)

/**
 * Explicit deterministic state machine for autonomous rover navigation.
 * Strictly prevents illegal state transitions and enforces safety invariants.
 */
class AutonomousNavigationStateMachine {

    private val _snapshot = MutableStateFlow(NavigationStateSnapshot())
    val snapshot: StateFlow<NavigationStateSnapshot> = _snapshot.asStateFlow()

    val currentState: NavigationState get() = _snapshot.value.state

    /**
     * Attempts a state transition. Enforces that high-priority safety states (ESTOP, FAULT)
     * cannot be casually exited without explicit recovery.
     */
    fun transition(
        to: NavigationState,
        reason: String,
        nowMs: Long = System.currentTimeMillis(),
    ): Boolean {
        val current = _snapshot.value.state
        if (current == to) return true

        // Invariant: ESTOP can only transition to IDLE or INITIALIZING via explicit reset
        if (current == NavigationState.ESTOP && to != NavigationState.IDLE && to != NavigationState.INITIALIZING) {
            return false
        }

        // Invariant: FAULT can only transition to IDLE
        if (current == NavigationState.FAULT && to != NavigationState.IDLE) {
            return false
        }

        _snapshot.value = NavigationStateSnapshot(
            state = to,
            reason = reason,
            timestampMs = nowMs,
        )
        return true
    }

    fun reset() {
        _snapshot.value = NavigationStateSnapshot(
            state = NavigationState.IDLE,
            reason = "State machine reset to IDLE",
        )
    }
}
