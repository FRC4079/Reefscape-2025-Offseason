package frc.robot.subsystems

import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.subsystems.Outtake.getCoralSensor
import frc.robot.utils.RobotParameters.IntakeParameters.STAR_FEEDER_PINGU
import frc.robot.utils.RobotParameters.IntakeParameters.WHEEL_FEEDER_PINGU
import frc.robot.utils.RobotParameters.MotorParameters.CORAL_FEEDER_ID
import frc.robot.utils.RobotParameters.MotorParameters.STAR_FEEDER_ID
import frc.robot.utils.RobotParameters.OuttakeParameters.outtakeState
import frc.robot.utils.emu.OuttakeState
import xyz.malefic.frc.pingu.motor.ControlType
import xyz.malefic.frc.pingu.motor.Mongu
import xyz.malefic.frc.pingu.motor.talonfx.TalonFXConfig

object Intake : SubsystemBase() {
    private val wheelFeederMotor =
        Mongu(TalonFX(CORAL_FEEDER_ID), control = ControlType.VELOCITY) {
            this as TalonFXConfig
            pingu = WHEEL_FEEDER_PINGU
            name = "Wheel Feeder Motor"
        }

    private val starFeederMotor =
        Mongu(TalonFX(STAR_FEEDER_ID), control = ControlType.VELOCITY) {
            this as TalonFXConfig
            pingu = STAR_FEEDER_PINGU
            name = "Star Feeder Motor"
        }

    init {
        intakeCoral()
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
        if (outtakeState == OuttakeState.STOWED && !getCoralSensor() && outtakeState != OuttakeState.CORAL_SHOOT) {
            intakeCoral()
        } else {
            stopMotors()
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
    fun intakeCoral() = setIntakeVelocity(30.0)

    /**
     * Starts the coral manipulator motors
     */
    fun reverseCoral() = setIntakeVelocity(-30.0)

    /**
     * Sets the speed of the algae intake motor.
     *
     * @param speed the desired speed to set for the intake motor
     */
    fun setIntakeVelocity(speed: Double) {
        wheelFeederMotor.move(speed)
        starFeederMotor.move(speed)
    }
}
