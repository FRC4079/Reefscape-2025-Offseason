package frc.robot.subsystems

import com.ctre.phoenix6.hardware.Pigeon2
import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.config.PIDConstants
import com.pathplanner.lib.controllers.PPHolonomicDriveController
import com.pathplanner.lib.path.PathConstraints
import com.pathplanner.lib.util.PathPlannerLogging
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator3d
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.subsystems.SuperStructure.currentState
import frc.robot.utils.RobotParameters.ControllerConstants.aacrn
import frc.robot.utils.RobotParameters.LiveRobotValues.robotPos
import frc.robot.utils.RobotParameters.MotorParameters.BACK_LEFT_CAN_CODER_ID
import frc.robot.utils.RobotParameters.MotorParameters.BACK_LEFT_DRIVE_ID
import frc.robot.utils.RobotParameters.MotorParameters.BACK_LEFT_STEER_ID
import frc.robot.utils.RobotParameters.MotorParameters.BACK_RIGHT_CAN_CODER_ID
import frc.robot.utils.RobotParameters.MotorParameters.BACK_RIGHT_DRIVE_ID
import frc.robot.utils.RobotParameters.MotorParameters.BACK_RIGHT_STEER_ID
import frc.robot.utils.RobotParameters.MotorParameters.FRONT_LEFT_CAN_CODER_ID
import frc.robot.utils.RobotParameters.MotorParameters.FRONT_LEFT_DRIVE_ID
import frc.robot.utils.RobotParameters.MotorParameters.FRONT_LEFT_STEER_ID
import frc.robot.utils.RobotParameters.MotorParameters.FRONT_RIGHT_CAN_CODER_ID
import frc.robot.utils.RobotParameters.MotorParameters.FRONT_RIGHT_DRIVE_ID
import frc.robot.utils.RobotParameters.MotorParameters.FRONT_RIGHT_STEER_ID
import frc.robot.utils.RobotParameters.MotorParameters.MAX_ANGULAR_SPEED
import frc.robot.utils.RobotParameters.MotorParameters.MAX_SPEED
import frc.robot.utils.RobotParameters.MotorParameters.PIDGEY_ID
import frc.robot.utils.RobotParameters.SwerveParameters.PhysicalParameters.kinematics
import frc.robot.utils.RobotParameters.SwerveParameters.PinguParameters
import frc.robot.utils.RobotParameters.SwerveParameters.PinguParameters.DRIVE_PINGU_AUTO
import frc.robot.utils.RobotParameters.SwerveParameters.PinguParameters.DRIVE_PINGU_TELE
import frc.robot.utils.RobotParameters.SwerveParameters.PinguParameters.ROTATIONAL_PINGU
import frc.robot.utils.RobotParameters.SwerveParameters.PinguParameters.X_PINGU
import frc.robot.utils.RobotParameters.SwerveParameters.PinguParameters.Y_PINGU
import frc.robot.utils.RobotParameters.SwerveParameters.Thresholds.CANCODER_VAL10
import frc.robot.utils.RobotParameters.SwerveParameters.Thresholds.CANCODER_VAL11
import frc.robot.utils.RobotParameters.SwerveParameters.Thresholds.CANCODER_VAL12
import frc.robot.utils.RobotParameters.SwerveParameters.Thresholds.CANCODER_VAL9
import frc.robot.utils.RobotParameters.SwerveParameters.Thresholds.IS_FIELD_ORIENTED
import frc.robot.utils.emu.Direction
import frc.robot.utils.emu.State.ScoreAlign
import frc.robot.utils.emu.State.TeleOpDrive
import frc.robot.utils.leftStickPosition
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber
import org.photonvision.EstimatedRobotPose
import xyz.malefic.frc.extension.getEstimatedPose
import xyz.malefic.frc.extension.to3d
import xyz.malefic.frc.extension.updateStdDev
import xyz.malefic.frc.extension.updateStdDev3d
import xyz.malefic.frc.pingu.LogPingu.log
import xyz.malefic.frc.pingu.LogPingu.logs
import xyz.malefic.frc.pingu.NetworkPingu
import java.util.Optional
import java.util.function.Predicate
import kotlin.math.abs
import kotlin.math.min

object Swerve : SubsystemBase() {
    private val poseEstimator: SwerveDrivePoseEstimator
    private val poseEstimator3d: SwerveDrivePoseEstimator3d
    private val field = Field2d()
    private val pidgey = Pigeon2(PIDGEY_ID)
    private val states = arrayOfNulls<SwerveModuleState>(4)
    private var setStates = arrayOfNulls<SwerveModuleState>(4)
    private val modules: Array<SwerveModule>

    // Auto Align Pingu Values
    private lateinit var networkPinguXAutoAlign: NetworkPingu
    private lateinit var networkPinguYAutoAlign: NetworkPingu
    private lateinit var networkPinguRotAutoAlign: NetworkPingu

    private val reefChooser: LoggedDashboardChooser<*>? = null
    private var desiredPoseForDriveToPoint: Pose2d
    private var maxVelocityOutputForDriveToPoint: Double
    private var maximumAngularVelocityForDriveToPoint: Double

    //
    //  Thread swerveLoggingThread =
    //      new Thread(
    //          () -> {
    //            while (DriverStation.isEnabled()) {
    //              logs("Swerve Module States", getModuleStates());
    //              try {
    //                Thread.sleep(100);
    //              } catch (InterruptedException e) {
    //                Thread.currentThread().interrupt();
    //                break;
    //              }
    //            }
    //          });
    // from feeder to the goal and align itself
    // The plan is for it to path towards it then we use a set path to align itself
    // with the goal and
    // be more accurate
    // Use this
    // https://pathplanner.dev/pplib-pathfinding.html#pathfind-then-follow-path
    var constraints: PathConstraints =
        PathConstraints(2.0, 3.0, Units.degreesToRadians(540.0), Units.degreesToRadians(720.0))

    /**
     * Creates a new instance of this SwerveSubsystem. This constructor is private since this class is
     * a Singleton. Code should use the [.getInstance] method to get the singleton instance.
     */
    init {
        this.modules = initializeModules()
        this.pidgey.reset()
        this.poseEstimator = initializePoseEstimator()
        this.poseEstimator3d = initializePoseEstimator3d()
        this.desiredPoseForDriveToPoint = Pose2d()
        this.maxVelocityOutputForDriveToPoint = Units.feetToMeters(10.0)
        this.maximumAngularVelocityForDriveToPoint = 0.0
        //    configureAutoBuilder();
        initializePathPlannerLogging()

        //    swerveLoggingThread.start();
        initializationAlignPingu()
    }

    /**
     * Initializes the swerve modules. Ensure the swerve modules are initialized in the same order as
     * in kinematics.
     *
     * @return SwerveModule[], An array of initialized SwerveModule objects.
     */
    private fun initializeModules(): Array<SwerveModule> =
        arrayOf(
            SwerveModule(
                FRONT_LEFT_DRIVE_ID,
                FRONT_LEFT_STEER_ID,
                FRONT_LEFT_CAN_CODER_ID,
                CANCODER_VAL9,
            ),
            SwerveModule(
                FRONT_RIGHT_DRIVE_ID,
                FRONT_RIGHT_STEER_ID,
                FRONT_RIGHT_CAN_CODER_ID,
                CANCODER_VAL10,
            ),
            SwerveModule(
                BACK_LEFT_DRIVE_ID,
                BACK_LEFT_STEER_ID,
                BACK_LEFT_CAN_CODER_ID,
                CANCODER_VAL11,
            ),
            SwerveModule(
                BACK_RIGHT_DRIVE_ID,
                BACK_RIGHT_STEER_ID,
                BACK_RIGHT_CAN_CODER_ID,
                CANCODER_VAL12,
            ),
        )

    /**
     * Initializes the SwerveDrivePoseEstimator. The SwerveDrivePoseEsimator estimates the robot's
     * position. This is based on a combination of the robot's movement and vision.
     *
     * @return SwerveDrivePoseEstimator, A new SwerveDrivePoseEstimator object.
     */
    private fun initializePoseEstimator(): SwerveDrivePoseEstimator =
        SwerveDrivePoseEstimator(
            kinematics,
            Rotation2d.fromDegrees(this.heading),
            this.modulePositions,
            Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)),
            VecBuilder.fill(0.02, 0.02, Units.degreesToRadians(5.0)),
            VecBuilder.fill(0.3, 0.3, Units.degreesToRadians(10.0)),
        )

    /**
     * Initializes the SwerveDrivePoseEstimator3d. The SwerveDrivePoseEsimator3d estimates the robot's
     * position in 3D space. This is based on a combination of the robot's movement and vision.
     *
     * @return SwerveDrivePoseEstimator3d, A new SwerveDrivePoseEstimator3d object.
     */
    private fun initializePoseEstimator3d(): SwerveDrivePoseEstimator3d =
        SwerveDrivePoseEstimator3d(
            kinematics,
            pidgey.getRotation3d(),
            this.modulePositions,
            Pose3d(0.0, 0.0, 0.0, Rotation3d(0.0, 0.0, 0.0)),
        )

    /**
     * Initializes the PathPlannerLogging. This allows the PathPlanner to log the robot's current
     * pose.
     */
    private fun initializePathPlannerLogging() {
        PathPlannerLogging.setLogCurrentPoseCallback { pose: Pose2d? -> field.robotPose = pose }
        PathPlannerLogging.setLogTargetPoseCallback { pose: Pose2d? ->
            field.getObject("target pose").pose = pose
        }
        PathPlannerLogging.setLogActivePathCallback { poses: MutableList<Pose2d?>? ->
            field.getObject("path").setPoses(poses)
        }
    }

    /**
     * Configures the AutoBuilder for autonomous driving. READ DOCUMENTATION TO PUT IN CORRECT VALUES
     * Allows PathPlanner to get pose and output robot-relative chassis speeds Needs tuning
     */
    fun configureAutoBuilder() {
        checkNotNull(PinguParameters.config)
        AutoBuilder.configure(
            { this.pose },
            { pose: Pose2d? -> this.newPose(pose!!) },
            { this.autoSpeeds },
            { chassisSpeeds: ChassisSpeeds? -> this.chassisSpeedsDrive(chassisSpeeds) },
            PPHolonomicDriveController(
                PIDConstants(7.6, 0.0, 0.0),
                PIDConstants(10.0, 0.0, 0.0),
            ),
            PinguParameters.config,
            {
                DriverStation.getAlliance().filter(Predicate { value: Alliance? -> value == Alliance.Red }).isPresent
            },
            this,
        )
    }

    private fun applySwerveState() {
        // SuperStructure.INSTANCE.getCurrentState() instanceof State.TeleOpDrive

        if (currentState is TeleOpDrive) {
            stickDrive(aacrn)
        } else if (currentState is ScoreAlign) {
            // Direction dir = ((State.ScoreAlign) SuperStructure.INSTANCE.getCurrentState()).getDir();
            val translationToDesiredPoint =
                desiredPoseForDriveToPoint.translation.minus(this.pose!!.translation)
            val linearDistance = translationToDesiredPoint.norm
            var frictionCoefficient = 0.1 // this is a guess
            if (linearDistance >= Units.inchesToMeters(0.5)) {
                frictionCoefficient *= MAX_SPEED
            }

            val directionOfTravel = translationToDesiredPoint.angle

            val velocityOutput: Double =
                if (DriverStation.isAutonomous()) {
                    min(
                        abs(DRIVE_PINGU_AUTO.pidController.calculate(linearDistance, 0.0)) + frictionCoefficient,
                        maxVelocityOutputForDriveToPoint,
                    )
                } else {
                    min(
                        abs(DRIVE_PINGU_TELE.pidController.calculate(linearDistance, 0.0)) + frictionCoefficient,
                        maxVelocityOutputForDriveToPoint,
                    )
                }
            val xComponent = velocityOutput * directionOfTravel.cos
            val yComponent = velocityOutput * directionOfTravel.sin

            val currentAngle = this.pose!!.rotation.degrees

            this.setDriveSpeeds(
                yComponent,
                xComponent,
                min(
                    abs(
                        ROTATIONAL_PINGU
                            .profiledPIDController
                            .calculate(
                                currentAngle,
                                desiredPoseForDriveToPoint.rotation.degrees,
                            ),
                    ),
                    MAX_ANGULAR_SPEED,
                ),
                false,
            )
        }
    }

    //    public void setSwerveState(SwerveRequest request) {
    //        this.setControl(request);
    //    }

    /**
     * This method is called periodically by the scheduler. It updates the pose estimator and
     * dashboard values.
     */
    override fun periodic() {
        updatePos()

        /*
         * Updates the robot position based on movement and rotation from the pidgey and
         * encoders.
         */
        poseEstimator.update(this.pidgeyRotation, this.modulePositions)
        poseEstimator3d.update(pidgey.getRotation3d(), this.modulePositions)

        field.robotPose = poseEstimator.estimatedPosition
        robotPos = poseEstimator.estimatedPosition

        logs {
            log("Swerve/Pidgey Yaw", this.pidgeyYaw)
            log("Swerve/Pidgey Heading", this.heading)
            log("Swerve/Pidgey Rotation", pidgey.pitch.valueAsDouble)
            log("Swerve/Pidgey Roll", pidgey.roll.valueAsDouble)
            log("Swerve/Pidgey Rotation2D", pidgey.rotation2d.degrees)
            log("Swerve/Robot Pose", field.robotPose)
            log("Swerve/Robot Pose 3D", poseEstimator3d.estimatedPosition)
            log("Swerve/Robot Pose 2D extra", robotPos)
        }
        applySwerveState()
    }

    /**
     * Updates the robot's position using vision measurements from PhotonVision. This method retrieves
     * the latest vision results, estimates the robot's pose, updates the standard deviations, and
     * adds the vision measurement to the pose estimator.
     */
    private fun updatePos() {
        PhotonVision.resultPairs?.isEmpty()?.let {
            if (!it) {
                PhotonVision
                    .resultPairs
                    ?.forEach { pair ->
                        val pose =
                            pair.getEstimatedPose(poseEstimator.estimatedPosition)
                        pair.updateStdDev(Optional.ofNullable<EstimatedRobotPose>(pose))
                        pair.updateStdDev3d(Optional.ofNullable<EstimatedRobotPose>(pose))
                        if (pose != null) {
                            val timestamp = pose.timestampSeconds
                            val visionMeasurement2d = pose.estimatedPose.toPose2d()
                            val visionMeasurement3d = pose.estimatedPose
                            poseEstimator.addVisionMeasurement(
                                visionMeasurement2d,
                                timestamp,
                                pair.first.currentStdDevs,
                            )
                            poseEstimator3d.addVisionMeasurement(
                                visionMeasurement3d,
                                timestamp,
                                pair.first.currentStdDevs3d,
                            )
                            robotPos = poseEstimator.estimatedPosition
                        }
                    }
            }
        }
    }

    /**
     * Sets the drive speeds for the swerve modules.3
     *
     * @param forwardSpeed The forward speed.
     * @param leftSpeed The left speed.
     * @param turnSpeed The turn speed.
     */
    fun setDriveSpeeds(
        forwardSpeed: Double,
        leftSpeed: Double,
        turnSpeed: Double,
    ) {
        setDriveSpeeds(forwardSpeed, leftSpeed, turnSpeed, IS_FIELD_ORIENTED)
    }

    /**
     * Sets the drive speeds for the swerve modules.
     *
     * @param forwardSpeed The forward speed.
     * @param leftSpeed The left speed.
     * @param turnSpeed The turn speed.
     * @param isFieldOriented Whether the robot is field oriented.
     */
    fun setDriveSpeeds(
        forwardSpeed: Double,
        leftSpeed: Double,
        turnSpeed: Double,
        isFieldOriented: Boolean,
    ) {
        logs {
            log("Swerve/Forward speed", forwardSpeed)
            log("Swerve/Left speed", leftSpeed)
            log("Swerve/Turn speed", turnSpeed)
        }

        // Converts to a measure that the robot aktualy understands
        var speeds =
            if (isFieldOriented) {
                ChassisSpeeds.fromFieldRelativeSpeeds(forwardSpeed, leftSpeed, turnSpeed, this.pidgeyRotation)
            } else {
                ChassisSpeeds(forwardSpeed, leftSpeed, turnSpeed)
            }

        logs<ChassisSpeeds?>("Swerve/Chassis Speeds", speeds)

        speeds = ChassisSpeeds.discretize(speeds, 0.02)

        val newStates: Array<SwerveModuleState?> = kinematics.toSwerveModuleStates(speeds)
        SwerveDriveKinematics.desaturateWheelSpeeds(newStates, MAX_SPEED)

        this.moduleStates = newStates
    }

    val pidgeyRotation: Rotation2d?
        /**
         * Gets the rotation of the Pigeon2 IMU.
         *
         * @return Rotation2d, The rotation of the Pigeon2 IMU.
         */
        get() = pidgey.rotation2d

    val heading: Double
        /**
         * Gets the heading of the robot.
         *
         * @return double, The heading of the robot.
         */
        get() = -pidgey.yaw.valueAsDouble

    val pidgeyYaw: Double
        /**
         * Gets the yaw of the Pigeon2 IMU.
         *
         * @return double, The yaw of the Pigeon2 IMU.
         */
        get() = pidgey.yaw.valueAsDouble

    /** Resets the Pigeon2 IMU.  */
    fun resetPidgey() {
        //    if (DriverStation.getAlliance().get() == Alliance.Blue) {
        //      pidgey.setYaw(0.0);
        //    } else {
        //      pidgey.setYaw(180.0);
        //    }
        // TODO Red side may have a starting angle of 180 instead of 0 for blue side which may mess
        // stuff up
        // Check to make sure 180 words or otherwise I can just flip everything
        // and ignore the flipping of the rotation cause it should be
        // just a note to future ppl: you should revert x maybe y controls for red side
        // odometry is seprate from pidgey so path planner should be fine? (it reverts paths
        // interestingly)
        pidgey.reset()
    }

    // om add docs pls
    fun flipPidgey() {
        pidgey.setYaw(180.0)
    }

    val pose: Pose2d?
        /**
         * Gets the current pose of the robot from the pose estimator.
         *
         * @return The current pose of the robot.
         */
        get() = poseEstimator.estimatedPosition

    /** Resets the pose of the robot to zero.  */
    fun zeroPose() {
        val heading = Rotation2d.fromDegrees(this.heading)
        val positions = this.modulePositions
        poseEstimator.resetPosition(heading, positions, Pose2d())
        poseEstimator3d.resetPosition(heading.to3d(this.pidgeyYaw), positions, Pose3d())
    }

    /**
     * Sets a new pose for the robot.
     *
     * @param pose The new pose.
     */
    fun newPose(pose: Pose2d) {
        poseEstimator.resetPosition(this.pidgeyRotation, this.modulePositions, pose)
        poseEstimator3d.resetPosition(pidgey.getRotation3d(), this.modulePositions, Pose3d(pose))
    }

    val autoSpeeds: ChassisSpeeds?
        /**
         * Gets the chassis speeds for autonomous driving.
         *
         * @return ChassisSpeeds, The chassis speeds for autonomous driving.
         */
        get() = kinematics.toChassisSpeeds(*this.moduleStates)

    /**
     * Drives the robot using chassis speeds.
     *
     * @param chassisSpeeds The chassis speeds.
     */
    fun chassisSpeedsDrive(chassisSpeeds: ChassisSpeeds?) {
        val newStates: Array<SwerveModuleState?> = kinematics.toSwerveModuleStates(chassisSpeeds)
        this.moduleStates = newStates
    }

    var moduleStates: Array<SwerveModuleState?>
        /**
         * Gets the states of the swerve modules.
         *
         * @return SwerveModuleState[], The states of the swerve modules.
         */
        get() {
            // TODO try returning new states maybe it is calling it too much why is why pp doesnt update
            val moduleStates = arrayOfNulls<SwerveModuleState>(4)
            for (i in modules.indices) {
                moduleStates[i] = modules[i].getState()
            }
            return moduleStates
        }

        /**
         * Sets the states of the swerve modules.
         *
         * @param states The states of the swerve modules.
         */
        set(states) {
            setStates = states
            for (i in states.indices) {
                modules[i].setState(states[i]!!)
            }
        }

    val setModuleStates: Array<SwerveModuleState?>
        /**
         * Gets the states of the swerve modules.
         *
         * @return SwerveModuleState[], The states of the swerve modules.
         */
        get() {
            val moduleStates = arrayOfNulls<SwerveModuleState>(4)
            System.arraycopy(setStates, 0, moduleStates, 0, setStates.size)
            return moduleStates
        }

    val modulePositions: Array<SwerveModulePosition?>
        /**
         * Gets the positions of the swerve modules.
         *
         * @return SwerveModulePosition[], The positions of the swerve modules.
         */
        get() {
            val positions = arrayOfNulls<SwerveModulePosition>(states.size)
            for (i in positions.indices) {
                positions[i] = modules[i].position
            }
            return positions
        }

    /** Stops all swerve modules.  */
    fun stop() {
        for (module in modules) {
            module.stop()
        }
    }

    /** Sets the PID constants for autonomous driving.  */
    fun setAutoPID() {
        for (module in modules) {
            module.setAutoPID()
        }
    }

    /** Sets the PID constants for teleoperated driving.  */
    fun setTelePID() {
        for (module in modules) {
            module.setTelePID()
            module.applyTelePIDValues()
        }
    }

    /** Resets the drive positions of all swerve modules.  */
    fun resetDrive() {
        for (module in modules) {
            module.resetDrivePosition()
        }
    }

    fun updateModuleTelePIDValues() {
        for (module in modules) {
            module.updateTelePID()
        }
    }

    val estimatedPose2D: Pose2d
        get() = poseEstimator3d.estimatedPosition.toPose2d()

    /**
     * The pose to path find to.
     *
     * @return Command, The command to path find to the goal.
     */
    fun pathFindToGoal(targetPose: Pose2d?): Command? = AutoBuilder.pathfindToPose(targetPose, constraints)

    private fun initializationAlignPingu() {
        networkPinguXAutoAlign =
            NetworkPingu(
                LoggedNetworkNumber("Tuning/Swerve/Align X P", X_PINGU.p),
                LoggedNetworkNumber("Tuning/Swerve/Align X I", X_PINGU.i),
                LoggedNetworkNumber("Tuning/Swerve/Align X D", X_PINGU.d),
            )

        networkPinguYAutoAlign =
            NetworkPingu(
                LoggedNetworkNumber("Tuning/Swerve/Align Y P", Y_PINGU.p),
                LoggedNetworkNumber("Tuning/Swerve/Align Y I", Y_PINGU.i),
                LoggedNetworkNumber("Tuning/Swerve/Align Y D", Y_PINGU.d),
            )

        networkPinguRotAutoAlign =
            NetworkPingu(
                LoggedNetworkNumber("Tuning/Swerve/Align Rot P", ROTATIONAL_PINGU.p),
                LoggedNetworkNumber("Tuning/Swerve/Align Rot I", ROTATIONAL_PINGU.i),
                LoggedNetworkNumber("Tuning/Swerve/Align Rot D", ROTATIONAL_PINGU.d),
            )
    }

    /**
     * Sets the desired pose for the robot to drive to, along with the maximum angular velocity and
     * direction for scoring.
     *
     * @param pose The target pose to drive to.
     * @param maximumAngularVelocityForDriveToPoint The maximum angular velocity allowed for driving
     * to the point.
     * @param direction The direction for score alignment.
     */
    fun setWantedDrivePose(
        pose: Pose2d,
        maximumAngularVelocityForDriveToPoint: Double,
        direction: Direction,
    ) {
        log("Swerve/Driving to scoring pose", pose)
        this.desiredPoseForDriveToPoint = pose
        SuperStructure + ScoreAlign(direction)
        this.maxVelocityOutputForDriveToPoint = Units.feetToMeters(10.0)
        this.maximumAngularVelocityForDriveToPoint = maximumAngularVelocityForDriveToPoint
    }

    /**
     * Drives the robot using the Xbox controller's joystick inputs.
     *
     * @param controller The XboxController providing joystick input.
     */
    fun stickDrive(controller: XboxController) {
        val position = controller.leftStickPosition()
        val rightX = controller.rightX
        val rotation = (if (abs(rightX) >= 0.1) -rightX * MAX_ANGULAR_SPEED else 0.0) * 0.5

        logs {
            log("X Joystick", position.first!!)
            log("Y Joystick", position.second!!)
            log("Rotation", rotation)
        }

        setDriveSpeeds(position.second!!, position.first!!, rotation)
    }
}
