package frc.robot.subsystems

import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC
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
import xyz.malefic.frc.pingu.alert.AlertPingu.add
import xyz.malefic.frc.pingu.control.NetworkPingu
import xyz.malefic.frc.pingu.control.Pingu
import xyz.malefic.frc.pingu.encoder.Engu
import xyz.malefic.frc.pingu.log.LogPingu.log
import xyz.malefic.frc.pingu.log.LogPingu.logs
import xyz.malefic.frc.pingu.motor.Mongu
import xyz.malefic.frc.pingu.motor.talonfx.TalonFXConfig
import xyz.malefic.frc.pingu.motor.talonfx.position
import xyz.malefic.frc.pingu.motor.talonfx.resetPosition
import xyz.malefic.frc.pingu.motor.talonfx.rotorVelocity
import xyz.malefic.frc.pingu.motor.talonfx.setControl
import xyz.malefic.frc.pingu.motor.talonfx.velocity

/** Represents a swerve module used in a swerve drive system.  */
class SwerveModule(
    val driveId: Int,
    val steerId: Int,
    canCoderID: Int,
    canCoderDriveStraightSteerSetPoint: Double,
) {
    private val driveMotor =
        Mongu(TalonFX(driveId)) {
            this as TalonFXConfig
            pingu = DRIVE_PINGU_TELE
            neutralMode = NeutralModeValue.Brake
            inverted = SwerveParameters.Thresholds.DRIVE_MOTOR_INVERTED
            currentLimits = DRIVE_SUPPLY_LIMIT to DRIVE_STATOR_LIMIT
            extraConfig = {
                Feedback.RotorToSensorRatio = DRIVE_MOTOR_GEAR_RATIO
            }
            name = "Swerve Drive Motor $driveId"
        }
    private val steerMotor =
        Mongu(TalonFX(steerId)) {
            this as TalonFXConfig
            pingu = STEER_PINGU_TELE
            neutralMode = NeutralModeValue.Brake
            inverted = SwerveParameters.Thresholds.STEER_MOTOR_INVERTED
            currentLimits = STEER_SUPPLY_LIMIT to null
            extraConfig = {
                ClosedLoopGeneral.ContinuousWrap = true
                Feedback.FeedbackRemoteSensorID = canCoderID
                Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder
                Feedback.RotorToSensorRatio = STEER_MOTOR_GEAR_RATIO
            }
            name = "Swerve Steer Motor $steerId"
        }
    private val canCoder = Engu(canCoderID)
    private val positionSetter = PositionTorqueCurrentFOC(0.0)
    private val velocitySetter = VelocityTorqueCurrentFOC(0.0)
    private val swerveModulePosition = SwerveModulePosition()

    var state: SwerveModuleState =
        SwerveModuleState(
            driveMotor.rotorVelocity / DRIVE_MOTOR_GEAR_RATIO * METERS_PER_REV,
            Rotation2d.fromRotations(canCoder.absolutePosition.valueAsDouble),
        )
        get() {
            field.angle = Rotation2d.fromRotations(canCoder.absolutePosition.valueAsDouble)
            field.speedMetersPerSecond =
                driveMotor.rotorVelocity / DRIVE_MOTOR_GEAR_RATIO * METERS_PER_REV
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
                log("Swerve/Drive actual sped", driveMotor.velocity)
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
        canCoder.configure {
            MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive
            MagnetSensor.MagnetOffset = ENCODER_OFFSET + canCoderDriveStraightSteerSetPoint
            MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1.0
        }

        driveVelocity = driveMotor.velocity
        drivePosition = driveMotor.position
        steerVelocity = steerMotor.velocity
        steerPosition = steerMotor.position

        val driveMotorConfig = driveMotor.configuration as TalonFXConfig

        networkPinguDrive =
            NetworkPingu(
                LoggedNetworkNumber("Tuning/Swerve/Drive P", driveMotorConfig.pingu.p),
                LoggedNetworkNumber("Tuning/Swerve/Drive I", driveMotorConfig.pingu.i),
                LoggedNetworkNumber("Tuning/Swerve/Drive D", driveMotorConfig.pingu.d),
                LoggedNetworkNumber("Tuning/Swerve/Drive V", driveMotorConfig.pingu.v),
            )

        val steerMotorConfig = steerMotor.configuration as TalonFXConfig

        networkPinguSteer =
            NetworkPingu(
                LoggedNetworkNumber("Tuning/Swerve/Steer P", steerMotorConfig.pingu.p),
                LoggedNetworkNumber("Tuning/Swerve/Steer I", steerMotorConfig.pingu.i),
                LoggedNetworkNumber("Tuning/Swerve/Steer D", steerMotorConfig.pingu.d),
                LoggedNetworkNumber("Tuning/Swerve/Steer V", steerMotorConfig.pingu.v),
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
            driveVelocity = driveMotor.velocity
            drivePosition = driveMotor.position
            steerVelocity = steerMotor.velocity
            steerPosition = steerMotor.position

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
        driveMotor.configure {
            this as TalonFXConfig
            this.pingu = pingu
        }
    }

    /**
     * Sets the PID configuration values for the steer motor using a Pingu configuration object.
     *
     * @param pingu A Pingu object containing PID and feedforward values (P, I, D, V)
     */
    fun setSteerPingu(pingu: Pingu) {
        steerMotor.configure {
            this as TalonFXConfig
            this.pingu = pingu
        }
    }

    /** Sets the PID values for teleoperation mode.  */
    fun setTelePID() {
        setDrivePingu(DRIVE_PINGU_TELE)
        setSteerPingu(STEER_PINGU_TELE)
    }

    /** Sets the PID values for autonomous mode.  */
    fun setAutoPID() {
        setDrivePingu(DRIVE_PINGU_AUTO)
        setSteerPingu(STEER_PINGU_AUTO)
    }

    /** Resets the drive motor position to zero.  */
    fun resetDrivePosition() = driveMotor.resetPosition(0.0)

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
        driveMotor.configure {
            this as TalonFXConfig
            this.pingu = networkPinguDrive.pingu
        }
        steerMotor.configure {
            this as TalonFXConfig
            this.pingu = networkPinguSteer.pingu
        }
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
