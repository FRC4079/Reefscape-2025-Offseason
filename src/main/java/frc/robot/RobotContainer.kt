package frc.robot

import com.pathplanner.lib.auto.AutoBuilder
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.commands.Kommand.moveElevatorState
import frc.robot.commands.Kommand.padElevator
import frc.robot.commands.Kommand.setElevatorState
import frc.robot.commands.Sequences.fullScoreAuto
import frc.robot.subsystems.Elevator
import frc.robot.subsystems.Elevator.setElevatorToBeSetState
import frc.robot.subsystems.Intake
import frc.robot.subsystems.SuperStructure
import frc.robot.utils.RobotParameters.ControllerConstants.aacrn
import frc.robot.utils.RobotParameters.ControllerConstants.testPad
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
import xyz.malefic.frc.emu.Button.LEFT_TRIGGER
import xyz.malefic.frc.emu.Button.RIGHT_BUMPER
import xyz.malefic.frc.emu.Button.RIGHT_STICK
import xyz.malefic.frc.emu.Button.X
import xyz.malefic.frc.emu.Button.Y
import xyz.malefic.frc.pingu.Bingu.bindings
import xyz.malefic.frc.pingu.CommandPingu

object RobotContainer {
    val networkChooser: SendableChooser<Command?>
    val calamityCow: XboxController = XboxController(1)

    init {
        Elevator.defaultCommand = padElevator(aacrn, calamityCow)

        CommandPingu.registerCommands {
            bind("ScoreL4Left", fullScoreAuto(LEFT))
            bind("ScoreL4Right", fullScoreAuto(RIGHT))
//            bind("HasPieceFalse", hasPieceFalse())
            bind("MoveElevatorL4Auto", moveElevatorState(L4))
            bind("MoveElevatorDefaultAuto", moveElevatorState(DEFAULT))
            bind("SetL1", setElevatorState(L1))
            bind("SetL2", setElevatorState(L2))
            bind("SetL3", setElevatorState(L3))
            bind("SetL4", setElevatorState(L4))
            bind("MoveElevatorDown", setElevatorState(DEFAULT))
        }

        networkChooser = AutoBuilder.buildAutoChooser()

        configureBindings()
    }

    private fun configureBindings() {
        aacrn.bindings {
            press(LEFT_BUMPER) {
                SuperStructure + State.ScoreAlign(LEFT)
            }
            press(RIGHT_BUMPER) {
                SuperStructure + State.ScoreAlign(RIGHT)
            }
            press(Y) {
                setElevatorToBeSetState(L4)
            }
            press(X) {
                setElevatorToBeSetState(L3)
            }
            press(B) {
                setElevatorToBeSetState(L2)
            }
            press(DPAD_UP) {
                setElevatorToBeSetState(L1)
            }
            press(A) {
                setElevatorToBeSetState(DEFAULT)
            }
            press(LEFT_TRIGGER) {
                Intake.intakeAlgae()
            }
            release(LEFT_TRIGGER) {
                Intake.stopIntake()
            }
            press(Button.RIGHT_TRIGGER) {
                when (SuperStructure.currentState) {
                    State.TeleOpDrive.Algae -> SuperStructure + State.Algae.Score
                    State.TeleOpDrive.Coral -> SuperStructure + State.ScoreManual
                    else -> { /* no-op */ }
                }
            }
            press(RIGHT_STICK) {
                visionDead = !visionDead
            }
        }

        testPad.bindings {
            press(A) {
                SuperStructure.driveToScoringPose(LEFT)
            }
            release(A) {
                SuperStructure.cancel()
            }
            press(B) {
                SuperStructure.driveToScoringPose(RIGHT)
            }
            release(B) {
                SuperStructure.cancel()
            }
        }
    }
}
