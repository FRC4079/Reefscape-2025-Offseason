package frc.robot.commands.auto

import edu.wpi.first.wpilibj2.command.InstantCommand
import frc.robot.subsystems.Outtake
import frc.robot.utils.RobotParameters.OuttakeParameters.outtakeState
import frc.robot.utils.emu.OuttakeState

class ScoreCoral : InstantCommand() {
    init {
        // each subsystem used by the command must be passed into the addRequirements() method
        addRequirements(Outtake)
    }

    override fun initialize() {
        outtakeState = OuttakeState.CORAL_SHOOT
    }
}
