package frc.robot.subsystems

import com.ctre.phoenix6.configs.CANcoderConfiguration
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.CANcoder
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.ctre.phoenix6.signals.SensorDirectionValue
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.subsystems.Elevator.setAlgaeLevel
import frc.robot.subsystems.Outtake.setOuttakeSpeed
import frc.robot.utils.RobotParameters.MotorParameters.OUTTAKE_OUTTAKE_MOTOR_ID
import frc.robot.utils.RobotParameters.MotorParameters.OUTTAKE_PIVOT_CANBORE_ID
import frc.robot.utils.RobotParameters.MotorParameters.OUTTAKE_PIVOT_MOTOR_ID
import frc.robot.utils.RobotParameters.OuttakeParameters
import frc.robot.utils.RobotParameters.OuttakeParameters.ALGAE_SENSOR_ID
import frc.robot.utils.RobotParameters.OuttakeParameters.CORAL_SENSOR_ID
import frc.robot.utils.RobotParameters.OuttakeParameters.algaeIntaking
import frc.robot.utils.RobotParameters.OuttakeParameters.outtakePivotState
import frc.robot.utils.RobotParameters.OuttakeParameters.outtakeState
import frc.robot.utils.RobotParameters.SwerveParameters.Thresholds.ENCODER_OFFSET
import frc.robot.utils.emu.ElevatorState
import frc.robot.utils.emu.OuttakePivotState
import frc.robot.utils.emu.OuttakeState
import frc.robot.utils.emu.State
import xyz.malefic.frc.pingu.AlertPingu.add
import xyz.malefic.frc.pingu.LogPingu.log
import xyz.malefic.frc.pingu.LogPingu.logs

/**
 * The Outtake class is a subsystem that interfaces with the arm system to provide control
 * over the arm motors. This subsystem is a Singleton, meaning that only one instance of this class
 * is created and shared across the entire robot code.
 */
object Outtake : SubsystemBase() {
    /** Creates a new end effector.  */
    private val pivotMotor: TalonFX = TalonFX(OUTTAKE_PIVOT_MOTOR_ID)
    private val outtakeMotor: TalonFX = TalonFX(OUTTAKE_OUTTAKE_MOTOR_ID)
    private val canbore = CANcoder(OUTTAKE_PIVOT_CANBORE_ID)

    private val voltageOut: VoltageOut
    private val voltagePos: PositionVoltage

    private val coralSensor: DigitalInput = DigitalInput(CORAL_SENSOR_ID)
    private val algaeSensor: DigitalInput = DigitalInput(ALGAE_SENSOR_ID)

    /**
     * Creates a new instance of this Outtake. This constructor is private since this class is a
     * Singleton. Code should use the [.getInstance] method to get the singleton instance.
     */
    init {
        val pivotMotorConfiguration = TalonFXConfiguration()
        val outtakeMotorConfiguration = TalonFXConfiguration()

        pivotMotorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake
        outtakeMotorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake

        pivotMotorConfiguration.Slot0.kP = OuttakeParameters.ALGAE_PINGU.p
        pivotMotorConfiguration.Slot0.kI = OuttakeParameters.ALGAE_PINGU.i
        pivotMotorConfiguration.Slot0.kD = OuttakeParameters.ALGAE_PINGU.d

        pivotMotorConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive
        outtakeMotorConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive

        pivotMotor.configurator.apply(pivotMotorConfiguration)
        outtakeMotor.configurator.apply(outtakeMotorConfiguration)

        pivotMotorConfiguration.CurrentLimits.SupplyCurrentLimit = 30.0
        pivotMotorConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true
        pivotMotorConfiguration.CurrentLimits.StatorCurrentLimit = 30.0
        pivotMotorConfiguration.CurrentLimits.StatorCurrentLimitEnable = true

        outtakeMotorConfiguration.CurrentLimits.SupplyCurrentLimit = 30.0
        outtakeMotorConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true
        outtakeMotorConfiguration.CurrentLimits.StatorCurrentLimit = 30.0
        outtakeMotorConfiguration.CurrentLimits.StatorCurrentLimitEnable = true

        pivotMotorConfiguration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0.0
        pivotMotorConfiguration.SoftwareLimitSwitch.ForwardSoftLimitEnable = false
        pivotMotorConfiguration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0
        pivotMotorConfiguration.SoftwareLimitSwitch.ReverseSoftLimitEnable = false

        pivotMotor.configurator.apply(pivotMotorConfiguration)
        outtakeMotor.configurator.apply(outtakeMotorConfiguration)

        val canCoderConfiguration = CANcoderConfiguration()

        canCoderConfiguration.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive
        canCoderConfiguration.MagnetSensor.MagnetOffset = ENCODER_OFFSET
        canCoderConfiguration.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1.0

        //    algaeManipulatorMotorConfiguration.MotorOutput.Inverted =
        // InvertedValue.Clockwise_Positive;
        //
        //    algaeManipulatorMotorConfiguration.SoftwareLimitSwitch =
        // algaeManipulatorMotorSoftLimitConfig;
        voltageOut = VoltageOut(0.0)
        voltagePos = PositionVoltage(0.0)

        pivotMotor.setPosition(0.0)

        add(pivotMotor, "algae pivot")
        add(outtakeMotor, "algae intake")
    }

    // This method will be called once per scheduler run
    override fun periodic() {
        outtakeState.block(this)

        setPivotState(if (outtakeState == OuttakeState.STOWED) OuttakePivotState.UP else OuttakePivotState.DOWN)

        when {
            outtakeState !in listOf(OuttakeState.CORAL_SHOOT, OuttakeState.ALGAE_SHOOT) -> {
                outtakeState =
                    when {
                        getCoralSensor() -> OuttakeState.CORAL_HOLD
                        getAlgaeSensor() -> OuttakeState.ALGAE_HOLD
                        else -> OuttakeState.STOWED
                    }
            }
            (outtakeState == OuttakeState.CORAL_SHOOT && !getCoralSensor()) ||
                (outtakeState == OuttakeState.ALGAE_SHOOT && !getAlgaeSensor()) -> {
                outtakeState = OuttakeState.STOWED
            }
            // otherwise, keep the current state (i.e., continue shooting)
        }

        logs {
            log("Algae/Algae Pivot Motor Position", this.pivotPosValue)
            log("Algae/Algae State", outtakePivotState.toString())
            log("Algae/IsAlgaeIntaking", algaeIntaking)
            log(
                "Algae/Disconnected algaeManipulatorMotor " + pivotMotor.deviceID,
                pivotMotor.isConnected,
            )
            log(
                "Algae/Algae Pivot Stator Current",
                pivotMotor.statorCurrent.valueAsDouble,
            )
            log(
                "Algae/Algae Pivot Supply Current",
                pivotMotor.supplyCurrent.valueAsDouble,
            )
            log(
                "Algae/Algae Pivot Stall Current",
                pivotMotor.motorStallCurrent.valueAsDouble,
            )
        }
    }

    /**
     * Sets the pivot state *
     *
     * @param state the state to set the algae pivot
     */
    fun setPivotState(state: OuttakePivotState) {
        pivotMotor.setControl(voltagePos.withPosition(state.pos))
    }

    val pivotPosValue: Double
        /**
         * Get the position of the end effector motor
         *
         * @return double, the position of the end effector motor
         */
        get() = pivotMotor.position.valueAsDouble

    /**
     * Sets the speed of the algae outtake motor.
     *
     * @param speed the desired speed to set for the intake motor
     */
    fun setOuttakeSpeed(speed: Double) {
        voltageOut.Output = speed
        outtakeMotor.setControl(voltageOut)
    }

    /** Stops the algae intake motor.  */
    fun stopMotors() {
        outtakeMotor.stopMotor()
        pivotMotor.stopMotor()
    }

    /**
     * Stows the outtake by stopping the outtake motor and moving the pivot to the UP position.
     */
    fun stow() {
        stopMotors() // TODO: is there anything else to do here?
    }

    /**
     * Stops the outtake motor.
     */
    fun stopOuttakeMotor() {
        outtakeMotor.stopMotor()
    }

    /**
     * Shoots coral by running the outtake at a fixed positive command.
     *
     * Uses `setOuttakeSpeed(30.0)` to eject coral for scoring.
     * @see setOuttakeSpeed
     */
    fun shootCoral() {
        setOuttakeSpeed(30.0)
    }

    /**
     * Reverses coral by running the outtake at a fixed negative command.
     *
     * Uses `setOuttakeSpeed(-30.0)` to shoot the coral into the intake.
     * @see setOuttakeSpeed
     */
    fun reverseCoral() {
        setOuttakeSpeed(-30.0)
    }

    /**
     * Sets the voltage output to -3.0 and controls the coral score motor to slow down the algae
     * scoring process.
     */
    fun slowAlgaeScoreMotors() = setOuttakeSpeed(-3.0)

    /**
     * Initiates the algae intake process. Sets the elevator to the algae level, starts the intake
     * motor, and marks the algaeIntaking flag as true.
     */
    fun intakeAlgae() {
        stopMotors()
        if (!setAlgaeLevel()) return
        setOuttakeSpeed(-30.0)
    }

    /**
     * Shoots algae by running the outtake at a fixed negative command.
     *
     * Uses `setOuttakeSpeed(-30.0)` to eject algae.
     * @see setOuttakeSpeed
     */
    fun shootAlgae() {
        // TODO: Weird elevator timing (in this file or somewhere else?)
        setOuttakeSpeed(-30.0)
        stopAlgaeIntake()
    }

    /**
     * Stops the algae intake process. Resets the elevator position to default, stops the algae intake
     * motor, and sets the algaeIntaking flag to false.
     */
    fun stopAlgaeIntake() {
        Elevator.setElevatorToBeSetState(ElevatorState.DEFAULT)
        stopMotors()
        SuperStructure + State.TeleOpDrive
    }

    /**
     * Gets the state of the coral sensor
     *
     * @return The state of the coral manipulator
     */
    fun getCoralSensor(): Boolean = !coralSensor.get()

    /**
     * Gets the state of the algae sensor
     *
     * @return The state of the algae sensor
     */
    fun getAlgaeSensor(): Boolean = !algaeSensor.get()
}
