package frc.robot.subsystems

import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.utils.RobotParameters.AlgaeManipulatorParameters.algaeIntaking
import frc.robot.utils.RobotParameters.CoralManipulatorParameters
import frc.robot.utils.RobotParameters.CoralManipulatorParameters.CORAL_FEEDER_PINGU
import frc.robot.utils.RobotParameters.CoralManipulatorParameters.coralScoring
import frc.robot.utils.RobotParameters.CoralManipulatorParameters.hasPiece
import frc.robot.utils.RobotParameters.CoralManipulatorParameters.outtakeState
import frc.robot.utils.RobotParameters.MotorParameters.CORAL_FEEDER_ID
import frc.robot.utils.RobotParameters.MotorParameters.CORAL_SCORE_ID
import frc.robot.utils.RobotParameters.MotorParameters.STAR_FEEDER_ID
import frc.robot.utils.emu.OuttakeState
import xyz.malefic.frc.pingu.AlertPingu.add
import xyz.malefic.frc.pingu.LogPingu.log
import xyz.malefic.frc.pingu.LogPingu.logs
import xyz.malefic.frc.pingu.VoltagePingu.setOutput

object Wrist : SubsystemBase() {
    // pivot motor
    // feeder motor
    // two sensors
    private val coralFeederMotor: TalonFX
    private val coralScoreMotor: TalonFX
    private val starFeederMotor: TalonFX

    private val voltageOut: VoltageOut
    private val voltageOutFeeder: VoltageOut

    private val coralSensor: DigitalInput

    private var motorsRunning = false

    /**
     * Creates a new instance of this CoralManipulatorSubsystem. This constructor is private since
     * this class is a Singleton. Code should use the [.getInstance] method to get the
     * singleton instance.
     */
    init {
        coralFeederMotor = TalonFX(CORAL_FEEDER_ID)
        coralScoreMotor = TalonFX(CORAL_SCORE_ID)
        starFeederMotor = TalonFX(STAR_FEEDER_ID)

        coralSensor = DigitalInput(CoralManipulatorParameters.CORAL_SENSOR_ID)

        val coralFeederConfiguration = TalonFXConfiguration()
        val coralScoreConfiguration = TalonFXConfiguration()
        val starFeederConfiguration = TalonFXConfiguration()

        coralFeederConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake
        coralScoreConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake
        starFeederConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast

        coralFeederConfiguration.Slot0.kP = CORAL_FEEDER_PINGU.p
        coralFeederConfiguration.Slot0.kI = CORAL_FEEDER_PINGU.i
        coralFeederConfiguration.Slot0.kD = CORAL_FEEDER_PINGU.d
        coralFeederConfiguration.Slot0.kV = CORAL_FEEDER_PINGU.v!!

        coralFeederMotor.getConfigurator().apply(coralFeederConfiguration)
        coralScoreMotor.getConfigurator().apply(coralFeederConfiguration)

        val upMotorCurrentConfig = CurrentLimitsConfigs()
        val coralScoreCurrentConfig = CurrentLimitsConfigs()
        val starFeederCurrentConfig = CurrentLimitsConfigs()

        upMotorCurrentConfig.SupplyCurrentLimit = 40.0
        upMotorCurrentConfig.SupplyCurrentLimitEnable = true
        upMotorCurrentConfig.StatorCurrentLimit = 40.0
        upMotorCurrentConfig.StatorCurrentLimitEnable = true

        coralScoreCurrentConfig.SupplyCurrentLimit = 40.0
        coralScoreCurrentConfig.SupplyCurrentLimitEnable = true
        coralScoreCurrentConfig.StatorCurrentLimit = 40.0
        coralScoreCurrentConfig.StatorCurrentLimitEnable = true

        starFeederCurrentConfig.SupplyCurrentLimit = 40.0
        starFeederCurrentConfig.SupplyCurrentLimitEnable = true
        starFeederCurrentConfig.StatorCurrentLimit = 40.0
        starFeederCurrentConfig.StatorCurrentLimitEnable = true

        coralFeederMotor.getConfigurator().apply(upMotorCurrentConfig)
        coralScoreMotor.getConfigurator().apply(coralScoreCurrentConfig)
        starFeederMotor.getConfigurator().apply(starFeederCurrentConfig)

        // on
        coralFeederConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive
        coralScoreConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive
        starFeederConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive

        coralScoreMotor.getConfigurator().apply(coralScoreConfiguration)
        coralFeederMotor.getConfigurator().apply(coralFeederConfiguration)
        starFeederMotor.getConfigurator().apply(starFeederConfiguration)

        voltageOut = VoltageOut(0.0)
        voltageOutFeeder = VoltageOut(0.0)

        add(coralFeederMotor, "coral feeder")
        add(coralScoreMotor, "coral score")
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
        coralFeederMotor.setControl(voltageOutFeeder)
        starFeederMotor.setControl(voltageOutFeeder)

        if (!algaeIntaking && !coralScoring) {
            if (!getCoralSensor() && !hasPiece) {
                outtakeState = OuttakeState.CORAL_INTAKE
            } else if (getCoralSensor() && !hasPiece) {
                // Stop the motors if the manipulator has a piece, but the sensor no longer
                // detects it
                outtakeState = OuttakeState.CORAL_HOLD // CORAL_SLOW
                setHasPiece(true)
            } else if (!getCoralSensor() && hasPiece) {
                outtakeState = OuttakeState.CORAL_HOLD
            } else {
                outtakeState = OuttakeState.CORAL_HOLD // CORAL_SLOW
            }
        }

        logs {
            log("Coral/Coral Sensor", getCoralSensor())
            log("Coral/Has Piece", hasPiece)
            log("Coral/Coral Scoring", coralScoring)
            log("Coral/motorsRunning", this.motorsRunning)
            log("Coral/Coral State", outtakeState.toString())
        }

        outtakeState.block.invoke()
    }

    /** Stops the coral manipulator motors  */
    fun stopMotors() {
        //    coralFeederMotor.stopMotor();
        coralScoreMotor.stopMotor()
        this.motorsRunning = false
    }

    /** Starts the coral manipulator motors  */
    fun startCoralIntake() {
        voltageOut.Output = 5.0
        coralScoreMotor.setControl(voltageOut)
        this.setHasPiece(false)
        //    isCoralIntaking = true;
    }

    /** Scores the coral motors  */
    fun scoreCoral() {
        voltageOut.Output = 3.0
        coralScoreMotor.setControl(voltageOut)
        this.motorsRunning = true
        this.setHasPiece(false)
    }

    /** Starts the coral manipulator motors  */
    fun slowCoralIntake() {
        coralScoreMotor.setControl(setOutput(4.0))
        //    coralFeederMotor.setControl(VoltagePingu.setOutput(4.0));
        this.setHasPiece(true)
    }

    /** Starts the coral manipulator motors  */
    fun reverseMotors() {
        coralFeederMotor.setControl(setOutput(-4.5))
        coralScoreMotor.setControl(setOutput(-4.5))
        starFeederMotor.setControl(setOutput(-4.5))
        this.motorsRunning = true
    }

    /**
     * Activates the algae intake process. This method stops the current motors, sets the voltage
     * output to 4.5, and starts the coral score motor to intake algae.
     */
    fun algaeIntake() {
        this.stopMotors()
        voltageOut.Output = -4.5
        coralScoreMotor.setControl(voltageOut)
        this.motorsRunning = true
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
        coralScoreMotor.setControl(voltageOut)
    }

    /**
     * Ejects the algae by setting the voltage output to 4.0 and controlling the coral score motor.
     */
    fun ejectAlgae() {
        voltageOut.Output = 4.0
        coralScoreMotor.setControl(voltageOut)
    }

    /**
     * Gets the state of the coral manipulator
     *
     * @return The state of the coral manipulator
     */
    fun getCoralSensor(): Boolean = !coralSensor.get()
}
