package frc.robot.commands

import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.commands.Kommand.moveToClosestCoralScore
import frc.robot.commands.Kommand.moveToClosestCoralScoreNotL4
import frc.robot.subsystems.Swerve
import frc.robot.utils.PathPingu.clearCoralScoringPositions
import frc.robot.utils.RobotParameters.ElevatorParameters.elevatorToBeSetState
import frc.robot.utils.RobotParameters.FieldParameters.RobotPoses.addCoralPosList
import frc.robot.utils.RobotParameters.LiveRobotValues.visionDead
import frc.robot.utils.RobotParameters.SwerveParameters.PinguParameters.PROFILE_CONSTRAINTS
import frc.robot.utils.RobotParameters.SwerveParameters.PinguParameters.ROTATIONAL_PINGU
import frc.robot.utils.RobotParameters.SwerveParameters.PinguParameters.X_PINGU
import frc.robot.utils.RobotParameters.SwerveParameters.PinguParameters.Y_PINGU
import frc.robot.utils.emu.Direction
import frc.robot.utils.emu.ElevatorState
import org.photonvision.PhotonCamera
import xyz.malefic.frc.pingu.LogPingu.log
import xyz.malefic.frc.pingu.LogPingu.logs
import xyz.malefic.frc.pingu.NetworkPingu

class AlignToPoseAuto(
    private val offsetSide: Direction,
) : Command() {
    private val yaw = 0.0
    private val y = 0.0
    private val dist = 0.0
    private var rotationalController: ProfiledPIDController? = null
    private var yController: ProfiledPIDController? = null
    private var xController: ProfiledPIDController? = null
    private var targetPose: Pose2d? = null
    private var currentPose: Pose2d? = null
    private var timer: Timer? = null
    private val offset = 0.0 // double offset is the left/right offset from the april tag to make it properly align
    var camera: PhotonCamera? = null

    private val networkPinguRotation: NetworkPingu? = null
    private val networkPinguY: NetworkPingu? = null
    private val networkPinguX: NetworkPingu? = null

    /** The initial subroutine of a command. Called once when the command is initially scheduled.  */
    override fun initialize() {
        // Update the list of coral scoring positions to the correct side (hopefully)
        currentPose = Swerve.pose2Dfrom3D

        timer = Timer()
        addRequirements(Swerve)

        clearCoralScoringPositions()
        addCoralPosList()
        currentPose = Swerve.pose2Dfrom3D

        if (elevatorToBeSetState == ElevatorState.L4) {
            targetPose = moveToClosestCoralScore(offsetSide, Swerve.pose2Dfrom3D)
        } else {
            targetPose = moveToClosestCoralScoreNotL4(offsetSide, Swerve.pose2Dfrom3D)
        }

        xController = X_PINGU.profiledPIDController
        xController!!.setTolerance(0.015)
        xController!!.setConstraints(PROFILE_CONSTRAINTS)
        xController!!.setGoal(targetPose!!.getX())
        xController!!.reset(currentPose!!.getX())

        yController = Y_PINGU.profiledPIDController
        yController!!.setTolerance(0.015)
        yController!!.setConstraints(PROFILE_CONSTRAINTS)
        yController!!.setGoal(targetPose!!.getY())
        yController!!.reset(currentPose!!.getY())

        rotationalController = ROTATIONAL_PINGU.profiledPIDController
        rotationalController!!.setTolerance(2.0)
        rotationalController!!.setConstraints(TrapezoidProfile.Constraints(5.0, 5.0))
        rotationalController!!.setGoal(targetPose!!.getRotation().getDegrees())
        rotationalController!!.reset(currentPose!!.getRotation().getDegrees())
        rotationalController!!.enableContinuousInput(-180.0, 180.0)
    }

    /**
     * The main body of a command. Called repeatedly while the command is scheduled. (That is, it is
     * called repeatedly until [.isFinished]) returns true.
     */
    override fun execute() {
        currentPose = Swerve.pose2Dfrom3D

        // TODO PLS CHECK BOTH SIDES AND BOTH APRIL TAG SIDES AND MAKE SURE IT ACTUALLY ALIGNS -SHAWN
        // TODO when aligning the x and y axes, you need to add sin/cos of the other axes angles and
        // feed it into the PID (probably need to test)
        // TODO the swerve should not need a negative anymore and we should not even have to check poses
        // theoretically??????? TEST THIS IN THE PIT ASAP
        if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
            if (targetPose!!.getX() < 4.5) {
                Swerve.setDriveSpeeds(
                    xController!!.calculate(currentPose!!.getX(), targetPose!!.getX()),
                    yController!!.calculate(currentPose!!.getY(), targetPose!!.getY()),
                    rotationalController!!.calculate(currentPose!!.getRotation().getDegrees()),
                    false,
                )
            } else {
                Swerve.setDriveSpeeds(
                    -xController!!.calculate(currentPose!!.getX(), targetPose!!.getX()),
                    -yController!!.calculate(currentPose!!.getY(), targetPose!!.getY()),
                    rotationalController!!.calculate(currentPose!!.getRotation().getDegrees()),
                    false,
                )
            }
        } else {
            // TODO MAKE A COPY OF ALIGNTOPOSE FOR AUTO WITH FIELD CENTRIC TURE!@!!!!@
            if (targetPose!!.getX() < 13) {
                Swerve.setDriveSpeeds(
                    xController!!.calculate(currentPose!!.getX()),
                    yController!!.calculate(currentPose!!.getY()),
                    rotationalController!!.calculate(currentPose!!.getRotation().getDegrees()),
                    false,
                )
            } else {
                Swerve.setDriveSpeeds(
                    -xController!!.calculate(currentPose!!.getX()),
                    -yController!!.calculate(currentPose!!.getY()),
                    rotationalController!!.calculate(currentPose!!.getRotation().getDegrees()),
                    false,
                )
            }
        }
        logs {
            log("AlignToPose/Current Pose", currentPose!!)
            log("AlignToPose/Target Pose", targetPose!!)
            log("AlignToPose/Rotational Error", rotationalController!!.getPositionError())
            log("AlignToPose/Y Error", yController!!.getPositionError())
            log("AlignToPose/X Error ", xController!!.getPositionError())
            log("AlignToPose/X Set ", xController!!.getSetpoint().position)
            log("AlignToPose/X Goal ", xController!!.getGoal().position)
            log("AlignToPose/Rotational Controller Setpoint", rotationalController!!.atSetpoint())
            log("AlignToPose/Y Controller Setpoint", yController!!.atSetpoint())
            log("AlignToPose/X Controller Setpoint ", xController!!.atSetpoint())
            log(
                "AlignToPose/X Set Speed ",
                xController!!.calculate(currentPose!!.getX(), targetPose!!.getX()),
            )
            log("AlignToPose/Y Set Speed ", yController!!.calculate(currentPose!!.getY()))
            log(
                "AlignToPose/Rot Set Speed ",
                rotationalController!!.calculate(currentPose!!.getRotation().getDegrees()),
            )
            log("AlignToPose/ X Set Pos", currentPose!!.getX())
            log("AlignToPose/ Y Set Pos", currentPose!!.getY())
            log("AlignToPose/ X Target Pos", targetPose!!.getX())
            log("AlignToPose/ Y Target Pos", targetPose!!.getY())
        }
    }

    /**
     * Returns whether this command has finished. Once a command finishes -- indicated by this method
     * returning true -- the scheduler will call its [.end] method.
     *
     *
     * Returning false will result in the command never ending automatically. It may still be
     * cancelled manually or interrupted by another command. Hard coding this command to always return
     * true will result in the command executing once and finishing immediately. It is recommended to
     * use [edu.wpi.first.wpilibj2.command.InstantCommand] for such an operation.
     *
     * @return whether this command has finished.
     */
    override fun isFinished(): Boolean {
        if ((rotationalController!!.atSetpoint() && yController!!.atSetpoint() && xController!!.atSetpoint()) ||
            visionDead
        ) {
            timer!!.start()
        } else {
            timer!!.reset()
        }
        return timer!!.hasElapsed(0.15)
        //    return false;
    }

    /**
     * The action to take when the command ends. Called when either the command finishes normally --
     * that is called when [.isFinished] returns true -- or when it is interrupted/canceled.
     * This is where you may want to wrap up loose ends, like shutting off a motor that was being used
     * in the command.
     *
     * @param interrupted whether the command was interrupted/canceled
     */
    override fun end(interrupted: Boolean) {
        Swerve.stop()
    } //  public void initializeLoggedNetworkPingu() {
    //    networkPinguRotation = new NetworkPingu(new
    // LoggedNetworkNumber("Tuning/AlignToPose/Rotational P", rotationalController.getP()), new
    // LoggedNetworkNumber("Tuning/AlignToPose/Rotational I", rotationalController.getI()), new
    // LoggedNetworkNumber("Tuning/AlignToPose/Rotational D", rotationalController.getD()));
    //    networkPinguY = new NetworkPingu(new LoggedNetworkNumber("Tuning/AlignToPose/Y P",
    // yController.getP()), new LoggedNetworkNumber("Tuning/AlignToPose/Y I", yController.getI()), new
    // LoggedNetworkNumber("Tuning/AlignToPose/Y D", yController.getD()));
    //    networkPinguX = new NetworkPingu(new LoggedNetworkNumber("Tuning/AlignToPose/X P",
    // xController.getP()), new LoggedNetworkNumber("Tuning/AlignToPose/X I", xController.getI()), new
    // LoggedNetworkNumber("Tuning/AlignToPose/X D", xController.getD()));
    //  }
}
