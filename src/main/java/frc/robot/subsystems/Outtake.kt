package frc.robot.subsystems

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.subsystems.Outtake.setOuttakeSpeed
import frc.robot.utils.RobotParameters.MotorParameters.ALGAE_INTAKE_MOTOR_ID
import frc.robot.utils.RobotParameters.MotorParameters.ALGAE_PIVOT_MOTOR_ID
import frc.robot.utils.RobotParameters.OuttakeParameters
import frc.robot.utils.RobotParameters.OuttakeParameters.ALGAE_SENSOR_ID
import frc.robot.utils.RobotParameters.OuttakeParameters.CORAL_SENSOR_ID
import frc.robot.utils.RobotParameters.OuttakeParameters.algaeIntaking
import frc.robot.utils.RobotParameters.OuttakeParameters.outtakePivotState
import frc.robot.utils.RobotParameters.OuttakeParameters.outtakeState
import frc.robot.utils.emu.OuttakePivotState
import frc.robot.utils.emu.OuttakeState
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
    private val pivotMotor: TalonFX = TalonFX(ALGAE_PIVOT_MOTOR_ID)
    private val outtakeMotor: TalonFX = TalonFX(ALGAE_INTAKE_MOTOR_ID)

    private val voltageOut: VoltageOut
    private val voltagePos: PositionVoltage

    private val coralSensor: DigitalInput = DigitalInput(CORAL_SENSOR_ID)
    private val algaeSensor: DigitalInput = DigitalInput(ALGAE_SENSOR_ID)

    /**
     * Creates a new instance of this Outtake. This constructor is private since this class is a
     * Singleton. Code should use the [.getInstance] method to get the singleton instance.
     */
    init {

        val algaePivotConfiguration = TalonFXConfiguration()
        val algaeIntakeConfiguration = TalonFXConfiguration()

        algaePivotConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake
        algaeIntakeConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake

        algaePivotConfiguration.Slot0.kP = OuttakeParameters.ALGAE_PINGU.p
        algaePivotConfiguration.Slot0.kI = OuttakeParameters.ALGAE_PINGU.i
        algaePivotConfiguration.Slot0.kD = OuttakeParameters.ALGAE_PINGU.d

        algaePivotConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive
        algaeIntakeConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive

        pivotMotor.configurator.apply(algaePivotConfiguration)
        outtakeMotor.configurator.apply(algaeIntakeConfiguration)

        algaePivotConfiguration.CurrentLimits.SupplyCurrentLimit = 30.0
        algaePivotConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true
        algaePivotConfiguration.CurrentLimits.StatorCurrentLimit = 30.0
        algaePivotConfiguration.CurrentLimits.StatorCurrentLimitEnable = true

        algaeIntakeConfiguration.CurrentLimits.SupplyCurrentLimit = 30.0
        algaeIntakeConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true
        algaeIntakeConfiguration.CurrentLimits.StatorCurrentLimit = 30.0
        algaeIntakeConfiguration.CurrentLimits.StatorCurrentLimitEnable = true

        algaePivotConfiguration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0.0
        algaePivotConfiguration.SoftwareLimitSwitch.ForwardSoftLimitEnable = false
        algaePivotConfiguration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0
        algaePivotConfiguration.SoftwareLimitSwitch.ReverseSoftLimitEnable = false

        pivotMotor.configurator.apply(algaePivotConfiguration)
        outtakeMotor.configurator.apply(algaeIntakeConfiguration)

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

        setPivotPos(outtakePivotState)

        when {
            outtakeState != OuttakeState.CORAL_SHOOT && outtakeState != OuttakeState.ALGAE_SHOOT -> {
                outtakeState =
                    if (getCoralSensor()) {
                        OuttakeState.CORAL_HOLD
                    } else if (getAlgaeSensor()) {
                        OuttakeState.ALGAE_HOLD
                    } else {
                        OuttakeState.STOWED
                    }
            }
            (outtakeState == OuttakeState.CORAL_SHOOT && !getCoralSensor()) ||
                (outtakeState == OuttakeState.ALGAE_SHOOT && !getAlgaeSensor()) -> {
                outtakeState = OuttakeState.STOWED
            }
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
    fun setPivotPos(state: OuttakePivotState) {
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
        stopOuttakeMotor()
        setPivotPos(OuttakePivotState.UP)
    }

    /**
     * Stops the outtake motor.
     */
    fun stopOuttakeMotor() {
        outtakeMotor.stopMotor()
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
