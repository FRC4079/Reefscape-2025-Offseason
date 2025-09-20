package frc.robot.utils.emu

sealed class State {
    // Top-level Auto option
    object Auto : State()

    // Tele-op drive with subcategories
    object TeleOpDrive : State()

    // Score with multiple levels and alignment options
    data class ScoreAlign(
        val dir: Direction,
    ) : State()

    object ScoreManual : State()
}
