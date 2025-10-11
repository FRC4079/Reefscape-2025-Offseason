package frc.robot.commands.auto

import edu.wpi.first.wpilibj2.command.InstantCommand
import frc.robot.subsystems.SuperStructure
import frc.robot.subsystems.Swerve
import frc.robot.utils.emu.Direction
import frc.robot.utils.emu.State

class Align(
    val direction: Direction,
) : InstantCommand() {
    init {
        addRequirements(Swerve)
    }

    override fun initialize() {
        SuperStructure + State.ScoreAlign(direction)
    }
}
