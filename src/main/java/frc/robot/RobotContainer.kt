package frc.robot

import com.pathplanner.lib.auto.AutoBuilder
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.commands.Kommand.drive
import frc.robot.commands.Kommand.hasPieceFalse
import frc.robot.commands.Kommand.moveElevatorState
import frc.robot.commands.Kommand.offVision
import frc.robot.commands.Kommand.onVision
import frc.robot.commands.Kommand.padElevator
import frc.robot.commands.Kommand.setElevatorState
import frc.robot.commands.sequencing.Sequences.fullScoreAuto
import frc.robot.subsystems.Elevator
import frc.robot.subsystems.SuperStructure
import frc.robot.subsystems.Swerve
import frc.robot.utils.RobotParameters.LiveRobotValues.visionDead
import frc.robot.utils.emu.Direction.LEFT
import frc.robot.utils.emu.Direction.RIGHT
import frc.robot.utils.emu.ElevatorState.DEFAULT
import frc.robot.utils.emu.ElevatorState.L1
import frc.robot.utils.emu.ElevatorState.L2
import frc.robot.utils.emu.ElevatorState.L3
import frc.robot.utils.emu.ElevatorState.L4
import frc.robot.utils.emu.State
import xyz.malefic.frc.emu.Button
import xyz.malefic.frc.emu.Button.A
import xyz.malefic.frc.emu.Button.B
import xyz.malefic.frc.emu.Button.DPAD_UP
import xyz.malefic.frc.emu.Button.LEFT_BUMPER
import xyz.malefic.frc.emu.Button.RIGHT_BUMPER
import xyz.malefic.frc.emu.Button.RIGHT_STICK
import xyz.malefic.frc.emu.Button.X
import xyz.malefic.frc.emu.Button.Y
import xyz.malefic.frc.extension.Kommand.cmd
import xyz.malefic.frc.pingu.Bingu.bindings
import xyz.malefic.frc.pingu.CommandPingu

class RobotContainer {
    val networkChooser: SendableChooser<Command?>
    val aacrn: XboxController = XboxController(0)
    val calamityCow: XboxController = XboxController(1)

    init {

        Elevator.getInstance().defaultCommand = padElevator(aacrn, calamityCow)
        Swerve.getInstance().defaultCommand = drive(aacrn)

        CommandPingu()
            .bind("ScoreL4Left", fullScoreAuto(LEFT))
            .bind("ScoreL4Right", fullScoreAuto(RIGHT))
            .bind("HasPieceFalse", hasPieceFalse())
            .bind("MoveElevatorL4Auto", moveElevatorState(L4))
            .bind("MoveElevatorDefaultAuto", moveElevatorState(DEFAULT))
            .bind("SetL1", setElevatorState(L1))
            .bind("SetL2", setElevatorState(L2))
            .bind("SetL3", setElevatorState(L3))
            .bind("SetL4", setElevatorState(L4))
            .bind("MoveElevatorDown", setElevatorState(DEFAULT))

        networkChooser = AutoBuilder.buildAutoChooser()

        configureBindings()
    }

    private fun configureBindings() {
        aacrn // Alignment
            .bindings {
                press(
                    LEFT_BUMPER,
                ) { cmd { SuperStructure.queueState(State.ScoreAlign(LEFT)) } }
                press(
                    RIGHT_BUMPER,
                ) { cmd { SuperStructure.queueState(State.ScoreAlign(RIGHT)) } }
                // Elevator States
                press(Y) {
                    cmd {
                        Elevator.setElevatorToBeSetState(
                            L4,
                        )
                    }
                }
                press(X) {
                    cmd {
                        Elevator.setElevatorToBeSetState(
                            L3,
                        )
                    }
                }
                press(B) {
                    cmd {
                        Elevator.setElevatorToBeSetState(
                            L2,
                        )
                    }
                }
                press(DPAD_UP) { cmd { Elevator.setElevatorToBeSetState(L1) } }
                // Stow Elevator
                press(A) {
                    cmd {
                        Elevator.getInstance().state = DEFAULT
                    }
                }
                // Intake
                // Algae
                press(Button.LEFT_TRIGGER) {
                    cmd {
                        Elevator.setAlgaeLevel()
                    }
                }
                press(RIGHT_STICK) {
                    if (visionDead) {
                        onVision()
                    } else {
                        offVision()
                    }
                }
            }
    }
}
