package frc.robot.subsystems

import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.commands.sequencing.Sequences
import frc.robot.utils.emu.State
import java.util.Queue

object SuperStructure : SubsystemBase() {
    private val swerve = Swerve.getInstance()
    private val algae = Algae.getInstance()
    private val coral = Coral.getInstance()
    private val elevator = Elevator.getInstance()

    private var currentState: State = State.TeleOpDrive.Base
    private var wantedState: ArrayDeque<State> = ArrayDeque()

    fun handleStateTransition() {
        when (wantedState.removeFirst()) {
            // TODO: add state transition handling
            else -> {}
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
        // This method will be called once per scheduler run
    }
}
