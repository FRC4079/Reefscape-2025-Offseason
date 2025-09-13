package frc.robot.subsystems

import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.commands.sequencing.Sequences
import frc.robot.utils.Pose.getDesiredScorePose
import frc.robot.utils.RobotParameters.ControllerConstants.aacrn
import frc.robot.utils.emu.Direction
import frc.robot.utils.emu.State

object SuperStructure : SubsystemBase() {
    var currentState: State = State.TeleOpDrive.Base
    private var wantedState: ArrayDeque<State> = ArrayDeque()

    /**
     * Adds a state to the wantedState queue using the plus operator.
     *
     * @param state The state to add.
     */
    operator fun plus(state: State) {
        wantedState.add(state)
    }

    /**
     * Adds a state to the wantedState queue.
     *
     * @param state The state to add.
     */
    fun addWantedState(state: State) {
        wantedState.add(state)
    }

    fun handleStateTransition() {
        when (wantedState.removeFirst()) {
            // TODO: add state transition handling
            else -> {}
        }
    }

    /**
     * Updates the `currentState` to the appropriate `TeleOpDrive` state based on sensor input.
     *
     * If the current state is a `TeleOpDrive` state, checks the status of the algae and coral sensors:
     * - If the algae sensor is active, sets the state to `TeleOpDrive.Algae`.
     * - If the coral sensors are active, sets the state to `TeleOpDrive.Coral`.
     * - Otherwise, sets the state to `TeleOpDrive.Base`.
     */
    fun handleDriveState() {
        if (currentState is State.TeleOpDrive) {
            currentState =
                when {
                    //bad logic change later u suck at coding u idiot
                    Outtake.getAlgaeSensor() -> State.TeleOpDrive.Algae
                    Outtake.getCoralSensor() -> State.TeleOpDrive.Coral
                    else -> State.TeleOpDrive.Base
                }
        }
    }

    fun applyState() {
        when (val state = currentState) {
            is State.TeleOpDrive -> {
                Swerve.stickDrive(aacrn)

                when (currentState) {
                    is State.TeleOpDrive -> {
                        Swerve.stickDrive(aacrn)
                        when (state) {
                            is State.TeleOpDrive.Algae -> {}
                            is State.TeleOpDrive.Coral -> {}
                            else -> {}
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
                        Outtake.shootAlgae()
                        // TODO: algae scoring sequence
                    }
                }
            }

            State.Climb -> TODO()
            State.ScoreManual -> TODO()
            State.Auto -> { /* no-op */ }
        } }

    fun driveToScoringPose(dir: Direction) {
        val poseToDriveTo = getDesiredScorePose(PhotonVision.getInstance().bestTargetID, dir)
        Swerve.setDesiredPoseForDriveToPointWithMaximumAngularVelocity(poseToDriveTo, 3.0, dir)
    }

    fun teleopScoringSequence() {
        // TODO: do the drive stuff here
    }

    /**
     * Cancels all current actions and resets the subsystem to its default state.
     *
     * Clears the wantedState queue, sets currentState to TeleOpDrive.Base,
     * and stops all relevant mechanisms (swerve, algae intake, coral motors).
     */
    fun cancel() {
        wantedState.clear()
        currentState = State.TeleOpDrive.Base
        Swerve.stop()
        Outtake.stopAlgaeMotor()
//        Intake.stopMotors()
    }

    override fun periodic() {
        handleDriveState()
        if (wantedState.isNotEmpty()) {
            handleStateTransition()
        }
        applyState()

    }
}
