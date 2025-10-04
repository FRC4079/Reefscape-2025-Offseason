@file:Suppress("ktlint:standard:no-wildcard-imports")

package frc.robot

import co.touchlab.kermit.Logger
import com.pathplanner.lib.auto.AutoBuilder
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.commands.Kommand.moveElevatorState
import frc.robot.commands.Kommand.setElevatorState
import frc.robot.commands.Sequences.fullScoreAuto
import frc.robot.subsystems.Elevator.setElevatorToBeSetState
import frc.robot.subsystems.Outtake
import frc.robot.subsystems.SuperStructure
import frc.robot.subsystems.Swerve
import frc.robot.utils.RobotParameters.ControllerConstants.aacrn
import frc.robot.utils.RobotParameters.ControllerConstants.testPad
import frc.robot.utils.RobotParameters.LiveRobotValues.visionDead
import frc.robot.utils.RobotParameters.OuttakeParameters.outtakeState
import frc.robot.utils.emu.Direction.LEFT
import frc.robot.utils.emu.Direction.RIGHT
import frc.robot.utils.emu.ElevatorState
import frc.robot.utils.emu.ElevatorState.*
import frc.robot.utils.emu.OuttakeState
import frc.robot.utils.emu.OuttakeState.*
import frc.robot.utils.emu.State
import frc.robot.utils.emu.State.ScoreManual
import xyz.malefic.frc.emu.Button
import xyz.malefic.frc.emu.Button.*
import xyz.malefic.frc.pingu.binding.Bingu.bindings
import xyz.malefic.frc.pingu.command.Commangu

object RobotContainer {
    val networkChooser: SendableChooser<Command?>

    init {
//        Elevator.defaultCommand = padElevator(testPad)

        Commangu.registerCommands {
            bind("ScoreL4Left", fullScoreAuto(LEFT))
            bind("ScoreL4Right", fullScoreAuto(RIGHT))
            // bind("HasPieceFalse", hasPieceFalse())
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
            release(LEFT_BUMPER) {
                SuperStructure.cancel()
            }
            press(RIGHT_BUMPER) {
                SuperStructure + State.ScoreAlign(RIGHT)
            }
            release(RIGHT_BUMPER) {
                SuperStructure.cancel()
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
            hold(LEFT_TRIGGER) {
                Outtake.intakeAlgae()
            }
            release(LEFT_TRIGGER) {
                Outtake.stopAlgaeIntake()
                if (Outtake.getAlgaeSensor()) {
                    outtakeState = ALGAE_HOLD
                    setElevatorToBeSetState(L2)
                }
            }
            press(RIGHT_TRIGGER) {
                when (outtakeState) {
                    ALGAE_HOLD -> outtakeState = ALGAE_SHOOT
                    CORAL_HOLD -> SuperStructure + ScoreManual
                    else -> { /* no-op */ }
                }
            }
            press(RIGHT_STICK) {
                visionDead = !visionDead
            }
            press(DPAD_DOWN) {
                Swerve.flipPidgey()
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
            press(Y) {
                setElevatorState(L4).schedule()
                outtakeState = CORAL_SHOOT
            }
            press(Button.X) {
                setElevatorState(DEFAULT).schedule()
            }
        }
    }
}
