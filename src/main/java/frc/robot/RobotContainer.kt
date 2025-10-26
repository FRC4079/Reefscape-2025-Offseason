@file:Suppress("ktlint:standard:no-wildcard-imports")

package frc.robot

import co.touchlab.kermit.Logger
import com.pathplanner.lib.auto.AutoBuilder
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import frc.robot.commands.AlignToPose
import frc.robot.commands.Kommand.moveElevatorState
import frc.robot.commands.Kommand.setElevatorState
import frc.robot.commands.Sequences.fullScoreAuto
import frc.robot.commands.auto.Align
import frc.robot.commands.auto.ScoreCoral
import frc.robot.subsystems.Elevator
import frc.robot.subsystems.Elevator.setElevatorToBeSetState
import frc.robot.subsystems.Intake
import frc.robot.subsystems.Outtake
import frc.robot.subsystems.Outtake.getCoralSensor
import frc.robot.subsystems.SuperStructure
import frc.robot.subsystems.Swerve
import frc.robot.utils.RobotParameters.ControllerConstants.aacrn
import frc.robot.utils.RobotParameters.ControllerConstants.testPad
import frc.robot.utils.RobotParameters.LiveRobotValues.slowMode
import frc.robot.utils.RobotParameters.LiveRobotValues.visionDead
import frc.robot.utils.RobotParameters.OuttakeParameters.outtakeState
import frc.robot.utils.emu.Direction
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
import kotlin.time.Instant

object RobotContainer {
//    val networkChooser: SendableChooser<Command?>
    var queuedState: OuttakeState = OuttakeState.STOWED

    init {
//        Elevator.defaultCommand = padElevator(testPad)

        Commangu.registerCommands {
//            bind("ScoreL4Left", fullScoreAuto(LEFT))
//            bind("ScoreL4Right", fullScoreAuto(RIGHT))
//            // bind("HasPieceFalse", hasPieceFalse())
//            bind("MoveElevatorL4Auto", moveElevatorState(L4))
//            bind("MoveElevatorDefaultAuto", moveElevatorState(DEFAULT))
//            bind("SetL1", setElevatorState(L1))
//            bind("SetL2", setElevatorState(L2))
//            bind("SetL3", setElevatorState(L3))
//            bind("SetL4", setElevatorState(L4))
//            bind("MoveElevatorDown", setElevatorState(DEFAULT))

            bind("AlignLeft", Align(Direction.LEFT))
            bind("AlignRight", Align(Direction.RIGHT))
            bind("StowElevator", setElevatorState(ElevatorState.DEFAULT))
            bind("MoveL2Auto", setElevatorState(ElevatorState.L2))
            bind("MoveL3Auto", setElevatorState(ElevatorState.L3))
            bind("MoveL4Auto", setElevatorState(ElevatorState.L4))
            bind("ScoreCoral", ScoreCoral())
        }

//        networkChooser = AutoBuilder.buildAutoChooser()

        configureBindings()
    }

    private fun configureBindings() {
        aacrn.bindings {
            press(DPAD_DOWN) {
                Swerve.resetPidgey()
            }
            press(DPAD_UP) {
                queuedState = outtakeState
                outtakeState = CORAL_REVERSE
            }
            release(DPAD_UP) {
                outtakeState = queuedState
                outtakeState
            }
            press(B) {
                if (Outtake.getCoralSensor()) {
                    setElevatorState(L3).schedule()
                } else {
                    setElevatorState(ElevatorState.ALGAE_HIGH).schedule()
                    outtakeState = OuttakeState.ALGAE_INTAKE
                }
            }
            press(X) {
                if (Outtake.getCoralSensor()) {
                    setElevatorState(L2).schedule()
                } else {
                    setElevatorState(ElevatorState.ALGAE_LOW).schedule()
                    outtakeState = OuttakeState.ALGAE_INTAKE
                }
            }
            press(Y) {
                setElevatorState(L4).schedule()
            }
            press(A) {
                setElevatorState(DEFAULT).schedule()
                outtakeState = STOWED
            }
            press(LEFT_TRIGGER) {
                visionDead = !visionDead
            }
            press(RIGHT_TRIGGER) {
                if (getCoralSensor()) {
                    outtakeState = OuttakeState.CORAL_SHOOT
                    slowMode = false
                } else if (outtakeState == OuttakeState.ALGAE_HOLD) {
                    outtakeState = OuttakeState.ALGAE_SHOOT
                    setElevatorState(ElevatorState.ALGAE_SHOOT).schedule()
                }
            }
//            press(LEFT_BUMPER) {
//                when (visionDead) {
//                    false -> AlignToPose(LEFT)
//                    true -> slowMode = true
//                }
//            }
//            release(LEFT_BUMPER) {
//                SuperStructure.cancel()
//            }
//            press(RIGHT_BUMPER) {
//                when (visionDead) {
//                    false -> AlignToPose(Direction.RIGHT)
//                    true -> slowMode = true
//                }
//            }
//            release(RIGHT_BUMPER) {
//                SuperStructure.cancel()
//            }
            press(LEFT_BUMPER) {
                SuperStructure + State.ScoreAlign(LEFT)
            }
            release(LEFT_BUMPER) {
                SuperStructure.cancel()
                println("Cancelled Left Align")
            }
            press(RIGHT_BUMPER) {
                SuperStructure + State.ScoreAlign(RIGHT)
            }
            release(RIGHT_BUMPER) {
                SuperStructure.cancel()
                println("Cancelled Right Align")
            }
        }
    }
}
