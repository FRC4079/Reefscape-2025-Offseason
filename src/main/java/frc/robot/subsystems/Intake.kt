package frc.robot.subsystems

import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.subsystems.Elevator.setAlgaeLevel
import frc.robot.utils.RobotParameters.MotorParameters.CORAL_FEEDER_ID
import frc.robot.utils.RobotParameters.MotorParameters.STAR_FEEDER_ID
import frc.robot.utils.RobotParameters.OuttakeParameters.CORAL_FEEDER_PINGU
import frc.robot.utils.RobotParameters.OuttakeParameters.algaeIntaking
import frc.robot.utils.RobotParameters.OuttakeParameters.coralScoring
import frc.robot.utils.RobotParameters.OuttakeParameters.hasPiece
import frc.robot.utils.RobotParameters.OuttakeParameters.outtakeState
import frc.robot.utils.emu.ElevatorState
import xyz.malefic.frc.pingu.AlertPingu.add
import xyz.malefic.frc.pingu.LogPingu.log
import xyz.malefic.frc.pingu.LogPingu.logs
import xyz.malefic.frc.pingu.VoltagePingu.setOutput

object Intake : SubsystemBase() {
    private val wheelFeederMotor: TalonFX
    private val starFeederMotor: TalonFX

    private val voltageOut: VoltageOut
    private val voltageOutFeeder: VoltageOut

    private var motorsRunning = false

    /**
     * Creates a new instance of this IntakeSubsystem. This constructor is private since this class is
     * a Singleton. Code should use the [.getInstance] method to get the singleton instance.
     */
    init {
        wheelFeederMotor = TalonFX(CORAL_FEEDER_ID)
        starFeederMotor = TalonFX(STAR_FEEDER_ID)

        val coralFeederConfiguration = TalonFXConfiguration()
        val starFeederConfiguration = TalonFXConfiguration()

        coralFeederConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake
        starFeederConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast

        coralFeederConfiguration.Slot0.kP = CORAL_FEEDER_PINGU.p
        coralFeederConfiguration.Slot0.kI = CORAL_FEEDER_PINGU.i
        coralFeederConfiguration.Slot0.kD = CORAL_FEEDER_PINGU.d
        coralFeederConfiguration.Slot0.kV = CORAL_FEEDER_PINGU.v!!

        val coralFeederCurrentConfig = CurrentLimitsConfigs()
        val starFeederCurrentConfig = CurrentLimitsConfigs()

        coralFeederCurrentConfig.SupplyCurrentLimit = 40.0
        coralFeederCurrentConfig.SupplyCurrentLimitEnable = true
        coralFeederCurrentConfig.StatorCurrentLimit = 40.0
        coralFeederCurrentConfig.StatorCurrentLimitEnable = true

        starFeederCurrentConfig.SupplyCurrentLimit = 40.0
        starFeederCurrentConfig.SupplyCurrentLimitEnable = true
        starFeederCurrentConfig.StatorCurrentLimit = 40.0
        starFeederCurrentConfig.StatorCurrentLimitEnable = true

        wheelFeederMotor.getConfigurator().apply(coralFeederCurrentConfig)
        starFeederMotor.getConfigurator().apply(starFeederCurrentConfig)

        // on
        coralFeederConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive
        starFeederConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive

        wheelFeederMotor.getConfigurator().apply(coralFeederConfiguration)
        starFeederMotor.getConfigurator().apply(starFeederConfiguration)

        voltageOut = VoltageOut(0.0)
        voltageOutFeeder = VoltageOut(0.0)

        add(wheelFeederMotor, "Coral Feeder")
        add(starFeederMotor, "Star Feeder Motor")
    }

    /**
     * If the coral sensor is triggered, set the hasPiece boolean to true. (hasPiece = true,
     * sensorDetect = true), motors spinning If the manipulator has a piece, but the sensor no longer
     * detects it, stop the motors. (hasPiece = true, sensorDetect = false), motors stop If the
     * manipulator should start, but the motors are not running, start the motors (hasPiece = false,
     * sensorDetect = false), motors spinning by setting if it has a piece to false, due to the fact
     * that the manipulator should not have a piece after the motors are started again.
     *
     *
     * The manipulator motors should be on by default, as per Aaron's request.
     */
    override fun periodic() {
        voltageOutFeeder.Output = 5.0
        wheelFeederMotor.setControl(voltageOutFeeder)
        starFeederMotor.setControl(voltageOutFeeder)

        logs {
            log("Coral/Coral Scoring", coralScoring)
            log("Coral/motorsRunning", motorsRunning)
            log("Coral/Coral State", outtakeState.toString())
        }

        outtakeState.block.invoke()
    }

    /**
     * Stops the coral manipulator motors
     */
    fun stopMotors() {
        wheelFeederMotor.stopMotor()
        starFeederMotor.stopMotor()
        motorsRunning = false
    }

    /**
     * Starts the coral manipulator motors
     */
    fun startCoralIntake() {
        voltageOut.Output = 5.0
        wheelFeederMotor.setControl(voltageOut)
        starFeederMotor.setControl(voltageOut)
        //    isCoralIntaking = true;
    }

    /**
     * Starts the coral manipulator motors
     */
    fun reverseMotors() {
        wheelFeederMotor.setControl(setOutput(-4.5))
        starFeederMotor.setControl(setOutput(-4.5))
        motorsRunning = true
    }

    /**
     * Initiates the algae intake process. Sets the elevator to the algae level, starts the intake
     * motor, and marks the algaeIntaking flag as true.
     */
    fun intakeAlgae() {
        stopMotors()
        setAlgaeLevel()
        setIntakeSpeed(4.5)
        algaeIntaking = true
    }

    /**
     * Sets the speed of the algae intake motor.
     *
     * @param speed the desired speed to set for the intake motor
     */
    fun setIntakeSpeed(speed: Double) {
        voltageOut.Output = speed
        wheelFeederMotor.setControl(voltageOut)
        starFeederMotor.setControl(voltageOut)
    }

    /**
     * Stops the algae intake process. Resets the elevator position to default, stops the algae intake
     * motor, and sets the algaeIntaking flag to false.
     */
    fun stopIntake() {
        Elevator.setElevatorToBeSetState(ElevatorState.DEFAULT)
        stopMotors()
        algaeIntaking = false
    }
}
