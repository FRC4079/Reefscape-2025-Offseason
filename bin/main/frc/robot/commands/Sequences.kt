package frc.robot.commands

import frc.robot.utils.emu.Direction

object Sequences {
    /**
     * Creates a sequence of commands to reset the scoring mechanism.
     *
     * @return A sequential command group.
     */
    fun resetScore() =
        Kommand.sequential {
//            +Kommand.setCoralState(OuttakeState.CORAL_REVERSE)
//            +Kommand.setElevatorState(ElevatorState.DEFAULT)
//            +Kommand.cancel()
//            +Kommand.hasPieceFalse()
        }

    /**
     * Creates a full scoring sequence for the current eleveator state in autonomous mode.
     *
     * Moving the elevator state to L4 and default is already accounted for in event markers.
     *
     * @param offsetSide The direction to offset the alignment.
     * @return A sequential command group.
     */
    fun fullScoreAuto(offsetSide: Direction) =
        Kommand.sequential {
//            +Kommand.align(offsetSide).withTimeout(1.25)
//            +Kommand.coralScoring()
//            +Kommand.setCoralState(OuttakeState.CORAL_REVERSE)
//            +Kommand.waitFor(0.5)
//            +Kommand.coralScoreFalse()
//            +Kommand.hasPieceFalse()
        }

    /**
     * Creates a full scoring sequence for teleoperated mode.
     *
     * @param offsetSide The direction to offset the alignment.
     * @return A sequential command group.
     */
    fun fullScore(offsetSide: Direction) =
        Kommand.sequential {
//            +Kommand.parallel {
//                +Kommand.moveElevatorState(RobotParameters.ElevatorParameters.elevatorToBeSetState)
//                +Kommand.align(offsetSide).withTimeout(2.0)
//            }
//            +Kommand.waitFor(0.1)
//            +Kommand.coralScoring()
//            +Kommand.setCoralState(OuttakeState.CORAL_REVERSE)
//            +Kommand.waitFor(0.5)
//            +Kommand.setElevatorState(ElevatorState.DEFAULT)
//            +Kommand.coralScoreFalse()
//            +Kommand.hasPieceFalse()
        }
}
