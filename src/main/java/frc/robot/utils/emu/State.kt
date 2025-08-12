package frc.robot.utils.emu

import frc.robot.utils.emu.State.Algae
import frc.robot.utils.emu.State.Algae.High
import frc.robot.utils.emu.State.Algae.Low
import frc.robot.utils.emu.State.Algae.Processor
import frc.robot.utils.emu.State.Algae.Score
import frc.robot.utils.emu.State.Auto
import frc.robot.utils.emu.State.Climb
import frc.robot.utils.emu.State.ScoreAlign
import frc.robot.utils.emu.State.ScoreAlign.L1
import frc.robot.utils.emu.State.ScoreAlign.L2
import frc.robot.utils.emu.State.ScoreAlign.L3
import frc.robot.utils.emu.State.ScoreAlign.L4
import frc.robot.utils.emu.State.ScoreManual
import frc.robot.utils.emu.State.TeleOpDrive
import kotlin.random.Random

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

        object Processor : Algae()
    }

    object Climb : State()
}
