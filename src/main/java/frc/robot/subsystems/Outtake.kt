package frc.robot.subsystems

import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.commands.Kommand.setElevatorState
import frc.robot.subsystems.Elevator.elevatorState
import frc.robot.subsystems.Outtake.setOuttakeSpeed
import frc.robot.utils.RobotParameters.MotorParameters.OUTTAKE_OUTTAKE_MOTOR_ID
import frc.robot.utils.RobotParameters.MotorParameters.OUTTAKE_PIVOT_MOTOR_ID
import frc.robot.utils.RobotParameters.OuttakeParameters.ALGAE_SENSOR_ID
import frc.robot.utils.RobotParameters.OuttakeParameters.CORAL_SENSOR_ID
import frc.robot.utils.RobotParameters.OuttakeParameters.OUTTAKE_PINGU
import frc.robot.utils.RobotParameters.OuttakeParameters.PIVOT_PINGU
import frc.robot.utils.RobotParameters.OuttakeParameters.outtakePivotState
import frc.robot.utils.RobotParameters.OuttakeParameters.outtakeState
import frc.robot.utils.emu.ElevatorState
import frc.robot.utils.emu.OuttakePivotState
import frc.robot.utils.emu.OuttakeState
import frc.robot.utils.emu.State
import xyz.malefic.frc.pingu.log.LogPingu.log
import xyz.malefic.frc.pingu.log.LogPingu.logs
import xyz.malefic.frc.pingu.motor.talonfx.TonguFX

/**
 * The Outtake class is a subsystem that interfaces with the arm system to provide control
 * over the arm motors. This subsystem is a Singleton, meaning that only one instance of this class
 * is created and shared across the entire robot code.
 */
object Outtake : SubsystemBase() {
    val shootTimer: Timer = Timer()
    val intakeTimer: Timer = Timer()
    val algaeTimer: Timer = Timer()

    var correctIntakingState: OuttakePivotState = OuttakePivotState.STOW

    private val voltageVelo: VelocityTorqueCurrentFOC = VelocityTorqueCurrentFOC(0.0)
    private val voltagePos: PositionVoltage = PositionVoltage(0.0)
    // Shawn was here

    /**
     * Pivot motor that controls the outtake pivot arm.
     */
    private val pivotMotor =
        TonguFX(OUTTAKE_PIVOT_MOTOR_ID, voltageVelo, { out -> this.withVelocity(out) }) {
            pingu = PIVOT_PINGU
            inverted = InvertedValue.Clockwise_Positive
            currentLimits = 30.0 to 30.0
            name = "Outtake Pivot Motor"
        }

    /**
     * Outtake motor responsible for intaking/shooting game pieces.
     */
    private val outtakeMotor =
        TonguFX(OUTTAKE_OUTTAKE_MOTOR_ID, voltageVelo, { out -> this.withVelocity(out) }) {
            pingu = OUTTAKE_PINGU
            neutralMode = NeutralModeValue.Brake
            inverted = InvertedValue.CounterClockwise_Positive
            currentLimits = 30.0 to 30.0
            name = "Outtake Outtake Motor"
        }

    private val coralSensor: DigitalInput = DigitalInput(CORAL_SENSOR_ID)
    private val algaeSensor: DigitalInput = DigitalInput(ALGAE_SENSOR_ID)

    // This method will be called once per scheduler run
    override fun periodic() {
        outtakeState.block(this)

        if (outtakeState == OuttakeState.CORAL_REVERSE) {
            reverseCoral()
            return
        }

        if (outtakeState == OuttakeState.ALGAE_HOLD && !getAlgaeSensor()) {
            outtakeState = OuttakeState.STOWED
        }

        // movePivotTo(if (outtakeState == OuttakeState.STOWED) OuttakePivotState.STOWED)

        // setOuttakeSpeed(testPadThree.leftY * 150)

//        println(intakeTimer.get())

        when (outtakeState) {
            OuttakeState.STOWED -> {
                movePivotTo(correctIntakingState)
                @Suppress("ktlint:standard:if-else-wrapping")
                if (!getCoralSensor()) {
                    setOuttakeSpeed(100.0)
                } else if (getCoralSensor()) {
                    intakeTimer.start()
                }

                if (intakeTimer.hasElapsed(0.1)) {
                    stopOuttakeMotor()
                    correctIntakingState = OuttakePivotState.CORAL_INTAKE
                }

                if (intakeTimer.hasElapsed(0.3)) {
                    setOuttakeSpeed(100.0)
                }

                if (intakeTimer.hasElapsed(3.1)) {
                    stopOuttakeMotor()
                    outtakeState = OuttakeState.CORAL_HOLD
                    correctIntakingState = OuttakePivotState.STOW
                }
                // otherwise we wait for 0.5 seconds to pass
            }

            OuttakeState.CORAL_HOLD -> {
                movePivotTo(correctIntakingState)
                if (!getCoralSensor()) {
                    outtakeState = OuttakeState.STOWED
                    correctIntakingState = OuttakePivotState.STOW
                }
            }

            OuttakeState.CORAL_SHOOT -> {
                when (elevatorState) {
                    ElevatorState.L4 -> {
                        movePivotTo(OuttakePivotState.CORAL_L4)
                        outtakePivotState = OuttakePivotState.CORAL_L4
                    }

                    ElevatorState.L3, ElevatorState.L2 -> {
                        movePivotTo(OuttakePivotState.CORAL_L23)
                        outtakePivotState = OuttakePivotState.CORAL_L23
                    }

                    ElevatorState.L1 -> {
                        movePivotTo(OuttakePivotState.CORAL_L1)
                        outtakePivotState = OuttakePivotState.CORAL_L1
                    }

                    else -> { /* no-op */ }
                }
            }

            OuttakeState.ALGAE_INTAKE -> {
                movePivotTo(OuttakePivotState.ALGAE_INTAKE)
                if (pivotMotor.position.valueAsDouble in
                    OuttakePivotState.ALGAE_INTAKE.pos - 0.1..OuttakePivotState.ALGAE_INTAKE.pos + 0.1
                ) {
                    if (getAlgaeSensor()) {
                        algaeTimer.start()
                    } else if (algaeTimer.hasElapsed(0.5)) {
                        outtakeState = OuttakeState.ALGAE_HOLD
                        algaeTimer.stop()
                        algaeTimer.reset()
                    }
                }
            }

            OuttakeState.ALGAE_HOLD -> {
                outtakePivotState = OuttakePivotState.ALGAE_HOLD
                setElevatorState(ElevatorState.DEFAULT).schedule()
            }
            else -> { /* no-op */ }
        }

        if (intakeTimer.hasElapsed(0.35) && intakeTimer.isRunning) {
            outtakeState = OuttakeState.CORAL_HOLD
            intakeTimer.stop()
            intakeTimer.reset()
            stopOuttakeMotor()
        }

        when {
            (outtakeState == OuttakeState.CORAL_SHOOT && !getCoralSensor()) -> {
                if (shootTimer.hasElapsed(0.5)) {
                    stopOuttakeMotor()
                    outtakeState = OuttakeState.STOWED
                    shootTimer.stop()
                    shootTimer.reset()
                    correctIntakingState = OuttakePivotState.STOW
                }
            }
//
//            (outtakeState == OuttakeState.ALGAE_SHOOT && !getAlgaeSensor()) -> {
//                outtakeState = OuttakeState.STOWED
//                stopOuttakeMotor()
//            }

            // otherwise, keep the current state (i.e., continue shooting)
        }

        logs {
            log("Outtake/Outtake Pivot Motor Position", pivotPosValue)
            log("Outtake/Outtake Pivot Motor Velocity", pivotMotor.velocity.valueAsDouble)
            log("Outtake/Outtake Pivot State State", outtakePivotState)
            log("Outtake/Outtake State", outtakeState)
            log(
                "Outtake/Disconnected algaeManipulatorMotor " + pivotMotor.deviceID,
                pivotMotor.isConnected,
            )
            log(
                "Outtake/Outtake Pivot Stator Current",
                pivotMotor.statorCurrent,
            )
            log(
                "Outtake/Outtake Pivot Supply Current",
                pivotMotor.supplyCurrent,
            )
            log(
                "Outtake/Outtake Pivot Stall Current",
                pivotMotor.motorStallCurrent,
            )
            log("Outtake/Coral Sensor", getCoralSensor())
            log("Outtake/Algae Sensor", getAlgaeSensor())
            log("Outtake/Voltage Velo", voltageVelo.Velocity)
            log("Outtake/Supplied voltage", outtakeMotor.supplyVoltage.value)
            log("Outtake/Supplied Current", outtakeMotor.supplyCurrent.value)
//            log (
//                "Outtake/Outtake Pivot Abs Encoder",
//                canbore.position.valueAsDouble
//            )
        }
    }

    /**
     * Moves the pivot
     *
     * @param state the state to set the algae pivot
     */
    fun movePivotTo(state: OuttakePivotState) {
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
     * Sets the speed of the coral outtake motor.
     *
     * @param speed the desired speed to set for the intake motor
     */
    fun setOuttakeSpeed(speed: Double) {
        outtakeMotor.setControl(voltageVelo.withVelocity(speed))
    }

    /** Stops the algae intake motor.  */
    fun stopMotors() {
        outtakeMotor.stopMotor()
        pivotMotor.stopMotor()
    }

    fun stopOuttakeMotor() = outtakeMotor.stopMotor()

    /**
     * Stows the outtake by stopping the outtake motor and moving the pivot to the UP position.
     */
    fun stow() {
//        stopMotors()
    }

    /**
     * Shoots coral by running the outtake at a fixed positive command.
     *
     * Uses `setOuttakeSpeed(100.0)` to eject coral for scoring.
     * @see setOuttakeSpeed
     */
    fun shootCoral() {
        if (pivotMotor.position.valueAsDouble in outtakePivotState.pos - 0.25..outtakePivotState.pos + 0.25 && Elevator.atState) {
            shootTimer.start()
            setOuttakeSpeed(30.0)
        }
    }

    /**
     * Reverses coral by running the outtake at a fixed negative command.
     *
     * Uses `setOuttakeSpeed(-30.0)` to shoot the coral into the intake.
     * @see setOuttakeSpeed
     */
    fun reverseCoral() {
        setOuttakeSpeed(-30.0) // TODO: Reverse Intake as well
    }

    /**
     * Sets the voltage output to -3.0 and controls the coral score motor to slow down the algae
     * scoring process.
     */
    fun slowAlgaeScoreMotors() = setOuttakeSpeed(75.0)

    /**
     * Initiates the algae intake process. Sets the elevator to the algae level, starts the intake
     * motor, and marks the algaeIntaking flag as true.
     */
    fun intakeAlgae() {
        setOuttakeSpeed(75.0)
        outtakePivotState = OuttakePivotState.ALGAE_INTAKE
    }

    /**
     * Shoots algae by running the outtake at a fixed negative command.
     *
     * Uses `setOuttakeSpeed(-30.0)` to eject algae.
     * @see setOuttakeSpeed
     */
    fun shootAlgae() {
        // TODO: Weird elevator timing (in this file or somewhere else?)
        if (Elevator.isAlgaeReadyForShoot()) {
            setOuttakeSpeed(-100.00)
        }

        if (Elevator.isAlgaeReadyForPivot()) {
            movePivotTo(OuttakePivotState.ALGAE_SHOOT)
        }

        if (!getAlgaeSensor()) {
            outtakeState = OuttakeState.STOWED
        }
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
