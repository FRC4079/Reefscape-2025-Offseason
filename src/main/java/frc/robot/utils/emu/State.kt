package frc.robot.utils.emu

sealed class State {
    // Top-level Auto option
    object Auto : State()

    // Tele-op drive with subcategories
    sealed class TeleOpDrive : State() {
        object Coral : TeleOpDrive()

        object Algae : TeleOpDrive()

        object Base : TeleOpDrive()
    }

    // Score with multiple levels and alignment options
    data class ScoreAlign(
        val dir: Direction,
    ) : State()

    object ScoreManual : State()

    // Algae with different options
    sealed class Algae : State() {
        object High : Algae()

        object Low : Algae()

        object Score : Algae()
    }

    object Climb : State()
}
