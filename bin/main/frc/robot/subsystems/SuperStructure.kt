package frc.robot.subsystems

import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.commands.Sequences
import frc.robot.utils.Pose.getDesiredScorePose
import frc.robot.utils.RobotParameters.ControllerConstants.aacrn
import frc.robot.utils.emu.Direction
import frc.robot.utils.emu.State

object SuperStructure : SubsystemBase() {
    var currentState: State = State.TeleOpDrive
    private var wantedState: ArrayDeque<State> = ArrayDeque()

    /**
     * Adds a state to the wantedState queue using the plus operator.
     *
     * @param state The state to add.
     */
    operator fun plus(state: State) {
        wantedState.add(state)
    }

    fun handleStateTransition() {
        when (wantedState.removeFirst()) {
            // TODO: add state transition handling
            else -> {}
        }
    }

    fun applyState() {
        when (val state = currentState) {
            is State.TeleOpDrive -> Swerve.stickDrive(aacrn)

            is State.ScoreAlign -> {
                val dir = state.dir
                Sequences.fullScore(dir) // TODO: actually implement a full score sequence
            }

            is State.Algae -> {
                when (state) {
                    is State.Algae.High -> {
                        // TODO: high algae sequence
                    }
                    is State.Algae.Low -> {
                        // TODO: low algae sequence
                    }
                }
            }

            State.ScoreManual -> TODO()
            State.Auto -> { /* no-op */ }
        }
    }

    fun driveToScoringPose(dir: Direction) {
        val poseToDriveTo = getDesiredScorePose(PhotonVision.bestTargetID, dir)
        Swerve.setWantedDrivePose(poseToDriveTo, 3.0, dir)
    }

    /**
     * Cancels all current actions and resets the subsystem to its default state.
     *
     * Clears the wantedState queue, sets currentState to TeleOpDrive.Base,
     * and stops all relevant mechanisms (swerve, algae intake, coral motors).
     */
    fun cancel() {
        wantedState.clear()
        currentState = State.TeleOpDrive
        Swerve.stop()
        Outtake.stopMotors()
        Intake.stopMotors()
    }

    override fun periodic() {
        if (wantedState.isNotEmpty()) {
            handleStateTransition()
        }
        applyState()
    }
}
