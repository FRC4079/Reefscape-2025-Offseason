package frc.robot.utils.emu

sealed class State {
    // Top-level Auto option (mostly no-op, handled externally)
    object Auto : State()

    // Tele-op drive, including manual alignment
    object TeleOpDrive : State()

    // Score alignment based on direction
    data class ScoreAlign(
        val dir: Direction,
    ) : State()
}
