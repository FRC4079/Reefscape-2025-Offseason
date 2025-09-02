package frc.robot.subsystems

import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.commands.sequencing.Sequences
import frc.robot.utils.Pose.getDesiredScorePose
import frc.robot.utils.RobotParameters.ControllerConstants.aacrn
import frc.robot.utils.emu.Direction
import frc.robot.utils.emu.State

object SuperStructure : SubsystemBase() {
    private val swerve = Swerve.getInstance()
    private val algae = Algae.getInstance()
    private val coral = Coral.getInstance()
    private val elevator = Elevator.getInstance()

    var currentState: State = State.TeleOpDrive.Base
    private var wantedState: ArrayDeque<State> = ArrayDeque()

    operator fun plus(state: State) {
        wantedState.add(state)
    }

    @JvmStatic
    fun resetState() {
        currentState = State.TeleOpDrive.Base
    }

    fun handleStateTransition() {
        when (wantedState.removeFirst()) {
            // TODO: add state transition handling
            else -> {}
        }
    }

    fun handleDriveState() {
        if (currentState is State.TeleOpDrive) {
            currentState =
                when {
                    Algae.getInstance().algaeSensor -> State.TeleOpDrive.Algae
                    Coral.getInstance().coralSensors -> State.TeleOpDrive.Coral
                    else -> State.TeleOpDrive.Base
                }
        }
    }

    fun applyState() {
        when (val state = currentState) {
            is State.TeleOpDrive -> {
                swerve.padDrive()

                when (currentState) {
                    is State.TeleOpDrive -> {
                        Swerve.getInstance().stickDrive(aacrn)
                        when (state) {
                            is State.TeleOpDrive.Algae -> {
                                Algae.getInstance().stopIntake()
                            }
                            is State.TeleOpDrive.Coral -> {
                                Coral.getInstance().stopMotors()
                            }
                            else -> { /* no-op */ }
                        }
                    }

                    else -> {}
                }
            }

            is State.ScoreAlign -> {
                val dir = state.dir // Ensure `dir` exists in `ScoreAlign`
                Sequences.fullScore(dir)
            }

            is State.Algae -> {
                when (state) {
                    is State.Algae.High -> {
                        // TODO: high algae sequence
                    }
                    is State.Algae.Low -> {
                        // TODO: low algae sequence
                    }
                    is State.Algae.Score -> {
                        Algae.getInstance().shootAlgae()
                        // TODO: algae scoring sequence
                    }
                }
            }

            State.Climb -> TODO()
            State.ScoreManual -> TODO()
            State.Auto -> { /* no-op */ }
            else -> { /* Handle unexpected states */ }
        }
    }

    fun setWantedState(state: State) {
        wantedState.add(state)
    }

    fun driveToScoringPose(dir: Direction) {
        val poseToDriveTo = getDesiredScorePose(PhotonVision.getInstance().bestTargetID, dir)
        swerve.setDesiredPoseForDriveToPointWithMaximumAngularVelocity(poseToDriveTo, 3.0, dir)

        // TODO: Thanks Jack in the Bots; we're at https://github.com/FRCTeam2910/2025CompetitionRobot-Public/blob/main/src/main/java/org/frc2910/robot/subsystems/Superstructure.java#L1392
    }

    fun teleopScoringSequence() {
        // fun do the drive stuff here
    }

    fun cancel() {
        wantedState.clear()
        currentState = State.TeleOpDrive.Base
        swerve.stop()
        algae.stopIntake()
        coral.stopMotors()
    }

    override fun periodic() {
        handleDriveState()
        if (wantedState.isNotEmpty()) {
            handleStateTransition()
        }
        applyState()
    }
}
