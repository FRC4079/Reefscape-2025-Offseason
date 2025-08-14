package frc.robot.subsystems

import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.utils.emu.AlgaeCounter
import frc.robot.utils.emu.State

object SuperStructure : SubsystemBase() {
    private val swerve = Swerve.getInstance()
    private val algae = Algae.getInstance()
    private val coral = Coral.getInstance()
    private val elevator = Elevator.getInstance()

    private var currentState: State = State.TeleOpDrive.Base
    private var wantedState: ArrayDeque<State> = ArrayDeque()

    @JvmStatic
    fun queueState(state: State) {
        wantedState.add(state)
    }

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
            if (Algae.getInstance().algaeSensor) {
                currentState = State.TeleOpDrive.Algae
            } else if (Coral.getInstance().coralSensor) {
                currentState = State.TeleOpDrive.Coral
            } else {
                currentState = State.TeleOpDrive.Base
            }
        }
    }

    fun applyState() {
        when (currentState) {
            is State.TeleOpDrive -> {
                // TODO: do drive
                when (currentState) {
                    is State.TeleOpDrive.Algae -> {
                        // TODO: idk
                    }
                    is State.TeleOpDrive.Coral -> {
                        // TODO: turn off intake
                    }
                    else -> {}
                }
            }
            is State.ScoreAlign -> {
                val dir = (currentState as State.ScoreAlign).dir
                // TODO: Full scoring sequence
            }
            is State.Algae ->
                when (currentState as State.Algae) {
                    is State.Algae.High -> {
                        // TODO: high algae sequence
                    }
                    is State.Algae.Low -> {
                        // TODO: low algae sequence
                    }
                    is State.Algae.Score -> {
                        // TODO: algae scoring sequence
                    }
                    is State.Algae.Processor -> {
                        // TODO: algae processor sequence
                    }
                }
            State.Climb -> {
                // TODO: climb sequence
            }
            State.ScoreManual -> TODO()
            State.Auto -> {}
        }
    }

    override fun periodic() {
        handleDriveState()
        if (!wantedState.isEmpty()) {
            handleStateTransition()
        }
        applyState()
    }
}
