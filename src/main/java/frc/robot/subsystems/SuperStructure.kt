package frc.robot.subsystems

import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.RobotContainer.aacrn
import frc.robot.commands.Kommand
import frc.robot.commands.sequencing.Sequences
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
                    Coral.getInstance().coralSensor -> State.TeleOpDrive.Coral
                    else -> State.TeleOpDrive.Base
                }
        }
    }

    fun applyState() {
        when (val state = currentState) {
            is State.TeleOpDrive -> {
<<<<<<< HEAD
                swerve.padDrive()
                when (currentState) {
=======
                Kommand.drive(aacrn)
                when (state) {
>>>>>>> b458a2cd3cab3cd0e0aa957d60ec8505997dd50a
                    is State.TeleOpDrive.Algae -> {
                        Algae.getInstance().stopIntake()
                    }
                    is State.TeleOpDrive.Coral -> {
                        Coral.getInstance().stopMotors()
                    }
                    else -> { /* no-op */ }
                }
            }

            is State.ScoreAlign -> {
                val dir = state.dir
                Sequences.fullScore(dir)
            }
            is State.Algae ->
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
            State.Climb -> TODO()
            State.ScoreManual -> TODO()
            State.Auto -> { /* no-op */ }
        }
    }

    override fun periodic() {
        handleDriveState()
        if (wantedState.isNotEmpty()) {
            handleStateTransition()
        }
        applyState()
    }
}
