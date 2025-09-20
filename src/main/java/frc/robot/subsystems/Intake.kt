package frc.robot.subsystems

import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.utils.RobotParameters.IntakeParameters.STAR_FEEDER_PINGU
import frc.robot.utils.RobotParameters.IntakeParameters.WHEEL_FEEDER_PINGU
import frc.robot.utils.RobotParameters.MotorParameters.CORAL_FEEDER_ID
import frc.robot.utils.RobotParameters.MotorParameters.STAR_FEEDER_ID
import frc.robot.utils.RobotParameters.OuttakeParameters.coralScoring
import frc.robot.utils.RobotParameters.OuttakeParameters.outtakeState
import xyz.malefic.frc.extension.configureWithDefaults
import xyz.malefic.frc.pingu.AlertPingu.add
import xyz.malefic.frc.pingu.LogPingu.log
import xyz.malefic.frc.pingu.LogPingu.logs
import xyz.malefic.frc.pingu.VoltagePingu.setOutput

object Intake : SubsystemBase() {
    private val wheelFeederMotor: TalonFX = TalonFX(CORAL_FEEDER_ID)
    private val starFeederMotor: TalonFX = TalonFX(STAR_FEEDER_ID)

    private val voltageOut: VoltageOut
    private val voltageOutFeeder: VoltageOut

    /**
     * Creates a new instance of this IntakeSubsystem. This constructor is private since this class is
     * a Singleton. Code should use the [.getInstance] method to get the singleton instance.
     */
    init {
        wheelFeederMotor.configureWithDefaults(WHEEL_FEEDER_PINGU)
        add(wheelFeederMotor, "Coral Feeder")

        starFeederMotor.configureWithDefaults(STAR_FEEDER_PINGU, neutralMode = NeutralModeValue.Coast)
        add(starFeederMotor, "Star Feeder Motor")

        voltageOut = VoltageOut(0.0)
        voltageOutFeeder = VoltageOut(0.0)
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
            log("Coral/Coral State", outtakeState.toString())
        }
    }

    /**
     * Stops the coral manipulator motors
     */
    fun stopMotors() {
        wheelFeederMotor.stopMotor()
        starFeederMotor.stopMotor()
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
}
