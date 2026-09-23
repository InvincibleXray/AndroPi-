package com.minesafety.roboeye.audio

/**
 * Robot auditory events for operational status and safety warnings.
 */
enum class AudioEvent {
    NORMAL_MOVEMENT,
    OBSTACLE_WARNING,
    EMERGENCY_STOP,
    LOW_BATTERY,
    EXPLORATION_CHIRP,
    MODE_CHANGE,
}

/**
 * Independent audio notification boundary.
 *
 * Never tightly coupled to motor control or navigation loops.
 */
interface AudioManager {
    /** Plays or schedules an auditory status cue. */
    fun playEvent(event: AudioEvent)

    /** Silences all active audio cues immediately. */
    fun mute()
}

/**
 * Default Android SoundPool / ToneGenerator placeholder implementation.
 */
class DefaultAudioManager : AudioManager {
    private var isMuted = false

    override fun playEvent(event: AudioEvent) {
        if (isMuted) return
        // Phase 2 audio synthesis / playback
    }

    override fun mute() {
        isMuted = true
    }
}
