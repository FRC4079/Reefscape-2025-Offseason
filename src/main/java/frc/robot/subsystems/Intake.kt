package frc.robot.subsystems

import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.subsystems.Outtake.getCoralSensor
import frc.robot.utils.RobotParameters.IntakeParameters.STAR_FEEDER_PINGU
import frc.robot.utils.RobotParameters.IntakeParameters.WHEEL_FEEDER_PINGU
import frc.robot.utils.RobotParameters.MotorParameters.CORAL_FEEDER_ID
import frc.robot.utils.RobotParameters.MotorParameters.STAR_FEEDER_ID
import frc.robot.utils.RobotParameters.OuttakeParameters.outtakeState
import frc.robot.utils.emu.OuttakeState
import xyz.malefic.frc.pingu.motor.talonfx.TonguFX

object Intake : SubsystemBase() {
    private val velocitySetter = VelocityTorqueCurrentFOC(0.0)

    private val wheelFeederMotor =
        TonguFX(CORAL_FEEDER_ID, velocitySetter, { out -> this.withVelocity(out) }) {
            pingu = WHEEL_FEEDER_PINGU
            neutralMode = NeutralModeValue.Brake
            inverted = InvertedValue.Clockwise_Positive
            name = "Wheel Feeder Motor"
        }
    private val starFeederMotor =
        TonguFX(STAR_FEEDER_ID, velocitySetter, { out -> this.withVelocity(out) }) {
            pingu = STAR_FEEDER_PINGU
            neutralMode = NeutralModeValue.Brake
            inverted = InvertedValue.CounterClockwise_Positive
            name = "Star Feeder Motor"
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
        if (outtakeState == OuttakeState.CORAL_REVERSE) {
            reverseCoral()
            return
        }
        if (!getCoralSensor() && outtakeState != OuttakeState.CORAL_SHOOT) {
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
//        wheelFeederMotor.move(speed.velocity)
//        starFeederMotor.move(speed.velocity)
        wheelFeederMotor.setControl(velocitySetter.withVelocity(speed))
        starFeederMotor.setControl(velocitySetter.withVelocity(speed))
    }
}
