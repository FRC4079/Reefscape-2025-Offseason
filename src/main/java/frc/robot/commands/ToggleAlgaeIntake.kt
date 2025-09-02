package frc.robot.commands

import frc.robot.subsystems.Elevator
import frc.robot.utils.RobotParameters.AlgaeManipulatorParameters.algaeCounter
import frc.robot.utils.RobotParameters.AlgaeManipulatorParameters.algaeIntaking
import frc.robot.utils.RobotParameters.AlgaeManipulatorParameters.algaePivotState
import frc.robot.utils.RobotParameters.CoralManipulatorParameters.coralState
import frc.robot.utils.RobotParameters.ElevatorParameters.elevatorToBeSetState
import frc.robot.utils.emu.AlgaeCounter.DEFAULT
import frc.robot.utils.emu.AlgaeCounter.HOLDING
import frc.robot.utils.emu.AlgaeCounter.INTAKE
import frc.robot.utils.emu.AlgaePivotState
import frc.robot.utils.emu.CoralState
import frc.robot.utils.emu.ElevatorState
import xyz.malefic.frc.extension.Kommand.cmd

val toggleIntakeAlgae =
    cmd {
        val elevator = Elevator.getInstance()
        when (algaeCounter) {
            INTAKE ->
                when (elevatorToBeSetState) {
                    ElevatorState.L2 -> {
                        algaePivotState = AlgaePivotState.DOWN
                        coralState = CoralState.ALGAE_INTAKE
                        elevator.state = ElevatorState.ALGAE_LOW
                        algaeIntaking = true
                    }
                    ElevatorState.L3 -> {
                        algaePivotState = AlgaePivotState.DOWN
                        coralState = CoralState.ALGAE_INTAKE
                        elevator.state = ElevatorState.ALGAE_HIGH
                        algaeIntaking = true
                    }
                    else -> {
                        algaeCounter = DEFAULT
                    }
                }

            HOLDING -> {
                // TODO: Handle holding
            }

            DEFAULT -> {
                algaePivotState = AlgaePivotState.UP
                coralState = CoralState.CORAL_INTAKE
                algaeIntaking = false
                elevator.state = ElevatorState.DEFAULT
            }
        }
        algaeCounter = algaeCounter.next
    }
