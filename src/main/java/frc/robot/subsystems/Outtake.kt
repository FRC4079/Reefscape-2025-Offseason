package frc.robot.subsystems

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.utils.RobotParameters.AlgaeManipulatorParameters
import frc.robot.utils.RobotParameters.AlgaeManipulatorParameters.algaeCounter
import frc.robot.utils.RobotParameters.AlgaeManipulatorParameters.algaeIntaking
import frc.robot.utils.RobotParameters.AlgaeManipulatorParameters.outtakePivotState
import frc.robot.utils.RobotParameters.CoralManipulatorParameters
import frc.robot.utils.RobotParameters.CoralManipulatorParameters.ALGAE_SENSOR_ID
import frc.robot.utils.RobotParameters.CoralManipulatorParameters.CORAL_SENSOR_ID
import frc.robot.utils.RobotParameters.CoralManipulatorParameters.coralScoring
import frc.robot.utils.RobotParameters.CoralManipulatorParameters.hasPiece
import frc.robot.utils.RobotParameters.CoralManipulatorParameters.outtakeState
import frc.robot.utils.RobotParameters.MotorParameters.ALGAE_INTAKE_MOTOR_ID
import frc.robot.utils.RobotParameters.MotorParameters.ALGAE_PIVOT_MOTOR_ID
import frc.robot.utils.emu.AlgaeCounter
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

        algaePivotConfiguration.Slot0.kP = AlgaeManipulatorParameters.ALGAE_PINGU.p
        algaePivotConfiguration.Slot0.kI = AlgaeManipulatorParameters.ALGAE_PINGU.i
        algaePivotConfiguration.Slot0.kD = AlgaeManipulatorParameters.ALGAE_PINGU.d

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
        setPivotPos(outtakePivotState)
        checkAlgaeSensor()

        if (!algaeIntaking && !coralScoring) {
            when {
                !getCoralSensor() && !hasPiece -> {
                    outtakeState = OuttakeState.CORAL_INTAKE
                }
                getCoralSensor() && !hasPiece -> {
                    // Stop the motors if the manipulator has a piece, but the sensor no longer detects it
                    outtakeState = OuttakeState.CORAL_HOLD
                    setHasPiece(true)
                }
                !getAlgaeSensor() && hasPiece -> {
                    outtakeState = OuttakeState.CORAL_HOLD
                }
                else -> {
                    outtakeState = OuttakeState.STOWED
                }
            }
        }

        if (getCoralSensor()) {
            outtakeState = OuttakeState.STOWED
        } else {
            outtakeState = OuttakeState.CORAL_HOLD
        }

        logs {
            log("Algae/Algae Pivot Motor Position", this.pivotPosValue)
            log("Algae/Algae State", outtakePivotState.toString())
            log("Algae/IsAlgaeIntaking", algaeIntaking)
            log("Algae/Algae counter", algaeCounter.toString())
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

    fun stow() {
        stopOuttakeMotor()
        setPivotPos(OuttakePivotState.UP)
    }

    fun stopOuttakeMotor() {
        outtakeMotor.stopMotor()
    }

    /**
     * Checks the state of the algae sensor. Sets the algaeCounter to HOLDING if the sensor is
     * triggered, otherwise sets it to DEFAULT.
     */
    fun checkAlgaeSensor() {
        algaeCounter =
            if (algaeSensor.get()) {
                AlgaeCounter.HOLDING
            } else {
                AlgaeCounter.DEFAULT
            }
    }

    fun shootAlgae() {
        setOuttakeSpeed(-30.0)
    }

    /**
     * Sets the state of whether the manipulator has a piece.
     *
     * @param hasPiece true if the manipulator has a piece, false otherwise
     */
    fun setHasPiece(hasPiece: Boolean) {
        CoralManipulatorParameters.hasPiece = hasPiece
    }

    /**
     * Sets the voltage output to -3.0 and controls the coral score motor to slow down the algae
     * scoring process.
     */
    fun slowAlgaeScoreMotors() {
        voltageOut.Output = -3.0
        outtakeMotor.setControl(voltageOut)
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
