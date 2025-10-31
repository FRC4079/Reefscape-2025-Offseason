package frc.robot.commands

import com.pathplanner.lib.auto.AutoBuilder
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import frc.robot.subsystems.Elevator
import frc.robot.subsystems.Intake
import frc.robot.subsystems.Swerve
import frc.robot.utils.PathPingu.findClosestScoringPosition
import frc.robot.utils.PathPingu.findClosestScoringPositionNotL4
import frc.robot.utils.RobotParameters.OuttakeParameters.outtakeState
import frc.robot.utils.RobotParameters.SwerveParameters.PinguParameters.PATH_CONSTRAINTS
import frc.robot.utils.emu.Direction
import frc.robot.utils.emu.ElevatorState
import frc.robot.utils.emu.OuttakeState
import xyz.malefic.frc.extension.Kommand.cmd
import xyz.malefic.frc.extension.Kommand.sequential
import xyz.malefic.frc.extension.Kommand.waitUntil
import kotlin.math.abs

/**
 * The [Kommand] object provides factory methods to create various commands
 * used in the robot's operation.
 *
 * This is called for instant commands instead of functions
 */
object Kommand {
    /**
     * Creates an [InstantCommand] to set the state of the elevator.
     *
     * @param state The desired state of the elevator.
     * @return An [InstantCommand] that sets the elevator state.
     */
    fun setElevatorState(state: ElevatorState) =
        cmd {
            Elevator.elevatorState = state
        }

    /**
     * Creates a [SequentialCommandGroup] to move the elevator to a specified state.
     *
     * @param state The desired state of the elevator.
     * @return A [SequentialCommandGroup] that moves the elevator to the specified state.
     */
    fun moveElevatorState(state: ElevatorState) =
        sequential {
            +cmd(Elevator) {
                Elevator.elevatorState = state
            }
            +waitUntil {
                abs(Elevator.elevatorPosAvg - state.pos) < 0.3
            }
        }

    /**
     * Creates an [InstantCommand] to set the state of the coral manipulator.
     *
     * @param state The desired state of the coral manipulator.
     * @return An [InstantCommand] that sets the coral manipulator state.
     */
    fun setCoralState(state: OuttakeState) = cmd { outtakeState = state }

    /**
     * Creates an [InstantCommand] to start the coral motors.
     *
     * @return An [InstantCommand] that starts the coral motors.
     */
    fun startCoralMotors() = cmd { Intake.intakeCoral() }

    /**
     * Creates an [InstantCommand] to reset the Pidgey sensor.
     *
     * @return An [InstantCommand] that resets the Pidgey sensor.
     */
    fun resetPidgey() = cmd { Swerve.resetPidgey() }

    /**
     * Creates an [InstantCommand] to flip the Pidgey sensor.
     *
     * @return An [InstantCommand] that flips the Pidgey sensor.
     */
    val flipPidgey = cmd { Swerve.flipPidgey() }

    /**
     * Creates an [InstantCommand] to set the teleoperation PID.
     *
     * @return An [InstantCommand] that sets the teleoperation PID.
     */
    fun setTelePid() = cmd { Swerve.setTelePID() }

    /**
     * Creates a pathfinding command to move to a specified pose.
     *
     * @param targetPose The target pose to move to.
     * @param endVelocity The end velocity for the pathfinding in m/s. Defaults to 0.0.
     * @return A command that performs the pathfinding operation.
     */
    fun createPathfindingCmd(
        targetPose: Pose2d,
        endVelocity: Double = 0.0,
    ): Command =
        AutoBuilder.pathfindToPose(
            targetPose,
            PATH_CONSTRAINTS,
            endVelocity,
        )

    /**
     * Creates a [PadElevator] command to control the elevator.
     *
     * @param controller The gaming controller used to move the elevator.
     * @return A [PadElevator] command to control the robot's elevator.
     */
    fun padElevator(controller: XboxController) = PadElevator(controller)

    /**
     * Creates a command to move the robot to the closest coral
     * scoring position in the specified coral direction.
     * @param direction The direction in which to find the closest scoring position.
     *
     * @return A command that performs the pathfinding operation.
     */
    fun moveToClosestCoralScore(
        direction: Direction,
        pose: Pose2d,
    ) = findClosestScoringPosition(pose, direction)

    /**
     * Creates a command to move the robot to the closest coral
     * scoring position in the specified coral direction.
     * @param direction The direction in which to find the closest scoring position.
     *
     * @return A command that performs the pathfinding operation.
     */
    fun moveToClosestCoralScoreNotL4(
        direction: Direction,
        pose: Pose2d,
    ) = findClosestScoringPositionNotL4(pose, direction)

    /**
     * Creates an [AlignToPose] command to align the robot to a specified pose.
     *
     * @param offsetSide The direction to offset the alignment.
     * @return An [AlignToPose] command to align the robot to the specified pose.
     */
    fun align(offsetSide: Direction) = AlignToPose(offsetSide)

    fun alignAuto(offsetSide: Direction) = AlignToPoseAuto(offsetSide)
}
