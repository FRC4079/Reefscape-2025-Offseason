package frc.robot.subsystems

import com.ctre.phoenix6.configs.CANcoderConfiguration
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC
import com.ctre.phoenix6.hardware.CANcoder
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.ctre.phoenix6.signals.SensorDirectionValue
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import frc.robot.utils.RobotParameters.MotorParameters.DRIVE_MOTOR_GEAR_RATIO
import frc.robot.utils.RobotParameters.MotorParameters.DRIVE_STATOR_LIMIT
import frc.robot.utils.RobotParameters.MotorParameters.DRIVE_SUPPLY_LIMIT
import frc.robot.utils.RobotParameters.MotorParameters.METERS_PER_REV
import frc.robot.utils.RobotParameters.MotorParameters.STEER_MOTOR_GEAR_RATIO
import frc.robot.utils.RobotParameters.MotorParameters.STEER_SUPPLY_LIMIT
import frc.robot.utils.RobotParameters.SwerveParameters
import frc.robot.utils.RobotParameters.SwerveParameters.PinguParameters.DRIVE_PINGU_AUTO
import frc.robot.utils.RobotParameters.SwerveParameters.PinguParameters.DRIVE_PINGU_TELE
import frc.robot.utils.RobotParameters.SwerveParameters.PinguParameters.STEER_PINGU_AUTO
import frc.robot.utils.RobotParameters.SwerveParameters.PinguParameters.STEER_PINGU_TELE
import frc.robot.utils.RobotParameters.SwerveParameters.Thresholds.ENCODER_OFFSET
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber
import xyz.malefic.frc.extension.configureWithDefaults
import xyz.malefic.frc.extension.setPingu
import xyz.malefic.frc.pingu.AlertPingu.add
import xyz.malefic.frc.pingu.LogPingu.log
import xyz.malefic.frc.pingu.LogPingu.logs
import xyz.malefic.frc.pingu.NetworkPingu
import xyz.malefic.frc.pingu.Pingu

/** Represents a swerve module used in a swerve drive system.  */
class SwerveModule(
    val driveId: Int,
    val steerId: Int,
    canCoderID: Int,
    canCoderDriveStraightSteerSetPoint: Double,
) {
    private val driveMotor = TalonFX(driveId)
    private val canCoder = CANcoder(canCoderID)
    private val steerMotor = TalonFX(steerId)
    private val positionSetter = PositionTorqueCurrentFOC(0.0)
    private val velocitySetter = VelocityTorqueCurrentFOC(0.0)
    private val swerveModulePosition = SwerveModulePosition()

    var state: SwerveModuleState = SwerveModuleState(0.0, Rotation2d.fromDegrees(0.0))
        get() {
            field.angle = Rotation2d.fromRotations(canCoder.absolutePosition.valueAsDouble)
            field.speedMetersPerSecond =
                driveMotor.rotorVelocity.valueAsDouble / DRIVE_MOTOR_GEAR_RATIO * METERS_PER_REV
            return field
        }
        set(desiredState) {
            // Get the current angle
            val currentAngle = Rotation2d.fromRotations(canCoder.absolutePosition.valueAsDouble)

            // Optimize the desired state based on current angle`
            desiredState.optimize(currentAngle)

            // Set the angle for the steer motor
            val angleToSet = desiredState.angle.rotations
            steerMotor.setControl(positionSetter.withPosition(angleToSet))

            // Set the velocity for the drive motor
            val velocityToSet: Double =
                (desiredState.speedMetersPerSecond * (DRIVE_MOTOR_GEAR_RATIO / METERS_PER_REV))
            driveMotor.setControl(velocitySetter.withVelocity(velocityToSet))

            // Log the actual and set values for debugging
            logs {
                log("Swerve/Drive actual sped", driveMotor.velocity.valueAsDouble)
                log("Swerve/Drive set sped", velocityToSet)
                log("Swerve/Steer actual angle", canCoder.absolutePosition.valueAsDouble)
                log("Swerve/Steer set angle", angleToSet)
                log("Swerve/Desired state after optimize", desiredState.angle.rotations)
            }

            // Update the state with the optimized values
            field = desiredState
        }

    private var driveVelocity: Double
    private var drivePosition: Double
    private var steerPosition: Double
    private var steerVelocity: Double
    private val driveConfigs: TalonFXConfiguration = TalonFXConfiguration()
    private val steerConfigs: TalonFXConfiguration = TalonFXConfiguration()

    private var networkPinguDrive: NetworkPingu
    private var networkPinguSteer: NetworkPingu

    /**
     * Constructs a new SwerveModule.
     *
     * @param driveId The ID of the drive motor.
     * @param steerId The ID of the steer motor.
     * @param canCoderID The ID of the CANcoder.
     * @param canCoderDriveStraightSteerSetPoint The set point for the CANcoder drive straight steer.
     */
    init {
        driveMotor.configureWithDefaults(
            DRIVE_PINGU_AUTO,
            neutralMode = NeutralModeValue.Brake,
            inverted = SwerveParameters.Thresholds.DRIVE_MOTOR_INVERTED,
            currentLimits = DRIVE_SUPPLY_LIMIT to DRIVE_STATOR_LIMIT,
        ) {
            Feedback.RotorToSensorRatio = DRIVE_MOTOR_GEAR_RATIO
        }

        steerMotor.configureWithDefaults(
            STEER_PINGU_AUTO,
            neutralMode = NeutralModeValue.Brake,
            inverted = SwerveParameters.Thresholds.STEER_MOTOR_INVERTED,
            currentLimits = STEER_SUPPLY_LIMIT to 0.0,
        ) {
            ClosedLoopGeneral.ContinuousWrap = true
            Feedback.FeedbackRemoteSensorID = canCoderID
            Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder
            Feedback.RotorToSensorRatio = STEER_MOTOR_GEAR_RATIO
        }

        canCoder.configureWithDefaults(
            sensorDirection = SensorDirectionValue.CounterClockwise_Positive,
            magnetOffset = ENCODER_OFFSET + canCoderDriveStraightSteerSetPoint,
            discontinuityPoint = 1.0,
        )

        driveMotor.configurator.apply(driveConfigs)
        steerMotor.configurator.apply(steerConfigs)

        driveVelocity = driveMotor.velocity.valueAsDouble
        drivePosition = driveMotor.position.valueAsDouble
        steerVelocity = steerMotor.velocity.valueAsDouble
        steerPosition = steerMotor.position.valueAsDouble

        networkPinguDrive =
            NetworkPingu(
                LoggedNetworkNumber("Tuning/Swerve/Drive P", driveConfigs.Slot0.kP),
                LoggedNetworkNumber("Tuning/Swerve/Drive I", driveConfigs.Slot0.kI),
                LoggedNetworkNumber("Tuning/Swerve/Drive D", driveConfigs.Slot0.kD),
                LoggedNetworkNumber("Tuning/Swerve/Drive V", driveConfigs.Slot0.kV),
            )
        networkPinguSteer =
            NetworkPingu(
                LoggedNetworkNumber("Tuning/Swerve/Steer P", steerConfigs.Slot0.kP),
                LoggedNetworkNumber("Tuning/Swerve/Steer I", steerConfigs.Slot0.kI),
                LoggedNetworkNumber("Tuning/Swerve/Steer D", steerConfigs.Slot0.kD),
                LoggedNetworkNumber("Tuning/Swerve/Steer V", steerConfigs.Slot0.kV),
            )

        initializeAlarms()
    }

    val position: SwerveModulePosition
        /**
         * Gets the current position of the swerve module.
         *
         * @return SwerveModulePosition, The current position of the swerve module.
         */
        get() {
            driveVelocity = driveMotor.velocity.valueAsDouble
            drivePosition = driveMotor.position.valueAsDouble
            steerVelocity = steerMotor.velocity.valueAsDouble
            steerPosition = steerMotor.position.valueAsDouble

            swerveModulePosition.angle = Rotation2d.fromRotations(canCoder.absolutePosition.valueAsDouble)
            swerveModulePosition.distanceMeters = drivePosition / DRIVE_MOTOR_GEAR_RATIO * METERS_PER_REV

            return swerveModulePosition
        }

    /** Stops the swerve module motors.  */
    fun stop() {
        steerMotor.stopMotor()
        driveMotor.stopMotor()
    }

    /**
     * Sets the PID configuration values for the drive motor using a Pingu configuration object.
     *
     * @param pingu A Pingu object containing PID and feedforward values (P, I, D, V)
     */
    fun setDrivePingu(pingu: Pingu) {
        driveConfigs.setPingu(pingu)
        driveMotor.configurator.apply(driveConfigs)
    }

    /**
     * Sets the PID configuration values for the steer motor using a Pingu configuration object.
     *
     * @param pingu A Pingu object containing PID and feedforward values (P, I, D, V)
     */
    fun setSteerPingu(pingu: Pingu) {
        steerConfigs.setPingu(pingu)
        driveMotor.configurator.apply(steerConfigs)
    }

    /** Sets the PID values for teleoperation mode.  */
    fun setTelePID() {
        setDrivePingu(DRIVE_PINGU_TELE)
        setSteerPingu(STEER_PINGU_TELE)
    }

    /** Sets the PID values for autonomous mode.  */
    fun setAutoPID() {
        setDrivePingu(DRIVE_PINGU_AUTO)
    }

    /** Resets the drive motor position to zero.  */
    fun resetDrivePosition() {
        driveMotor.setPosition(0.0)
    }

    /**
     * Updates the PID values for teleoperation mode. This method retrieves the PID values from the
     * network Pingu configuration objects and sets these values for the drive and steer motors.
     */
    fun updateTelePID() {
        DRIVE_PINGU_TELE.setPID(networkPinguDrive)
        STEER_PINGU_TELE.setPID(networkPinguSteer)

        applyTelePIDValues()
    }

    /**
     * Applies the PID configuration values for teleoperation mode. This method sets the PID values
     * for both the drive and steer motors using the network Pingu configuration objects and applies
     * these configurations to the respective motors.
     */
    fun applyTelePIDValues() {
        driveConfigs.setPingu(networkPinguDrive.pingu)
        steerConfigs.setPingu(networkPinguSteer.pingu)

        driveMotor.configurator.apply(driveConfigs)
        steerMotor.configurator.apply(steerConfigs)
    }

    /**
     * Initializes alerts for disconnected components.
     *
     * @param driveId The ID of the drive motor.
     * @param steerId The ID of the steer motor.
     */
    fun initializeAlarms() {
        add(driveMotor, "drive motor $driveId")
        add(steerMotor, "steer motor $steerId")
        add(canCoder)
    }
}
