package frc.robot.subsystems

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.signals.InvertedValue
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.utils.RobotParameters.ElevatorParameters.ELEVATOR_MAGIC_PINGU
import frc.robot.utils.RobotParameters.ElevatorParameters.ELEVATOR_PINGU
import frc.robot.utils.RobotParameters.ElevatorParameters.ELEVATOR_SOFT_LIMIT_DOWN
import frc.robot.utils.RobotParameters.ElevatorParameters.ELEVATOR_SOFT_LIMIT_UP
import frc.robot.utils.RobotParameters.ElevatorParameters.elevatorToBeSetState
import frc.robot.utils.RobotParameters.MotorParameters.ELEVATOR_MOTOR_LEFT_ID
import frc.robot.utils.RobotParameters.MotorParameters.ELEVATOR_MOTOR_RIGHT_ID
import frc.robot.utils.emu.ElevatorMotor
import frc.robot.utils.emu.ElevatorState
import frc.robot.utils.emu.ElevatorState.L2
import frc.robot.utils.emu.ElevatorState.L3
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber
import xyz.malefic.frc.pingu.log.LogPingu.log
import xyz.malefic.frc.pingu.log.LogPingu.logs
import xyz.malefic.frc.pingu.motor.talonfx.TonguFX

/**
 * The ElevatorSubsystem class is a Singleton to control the elevator motors on the robot. The class
 * sets the motor positions, gets the motor positions, stops the motors, and toggles the soft stop
 * for the elevator motor.
 */
object Elevator : SubsystemBase() {
    private val voltagePos: PositionVoltage = PositionVoltage(0.0)

    private val voltageOut: VoltageOut = VoltageOut(0.0)

    private val elevatorMotorLeft =
        TonguFX(ELEVATOR_MOTOR_LEFT_ID, voltagePos, { out -> this.withPosition(out) }) {
            pingu = ELEVATOR_PINGU
            inverted = InvertedValue.CounterClockwise_Positive
            softLimits = ELEVATOR_SOFT_LIMIT_UP to ELEVATOR_SOFT_LIMIT_DOWN
            currentLimits = null
            dutyCycleNeutralDeadband = 0.1
            motionMagicPingu = ELEVATOR_MAGIC_PINGU
            name = "Elevator Left Motor"
        }
    private val elevatorMotorRight =
        TonguFX(ELEVATOR_MOTOR_RIGHT_ID, voltagePos, { out -> this.withPosition(out) }) {
            pingu = ELEVATOR_PINGU
            softLimits = ELEVATOR_SOFT_LIMIT_UP to ELEVATOR_SOFT_LIMIT_DOWN
            currentLimits = null
            dutyCycleNeutralDeadband = 0.1
            motionMagicPingu = ELEVATOR_MAGIC_PINGU
            name = "Elevator Right Motor"
        }

    private lateinit var elevatorP: LoggedNetworkNumber
    private lateinit var elevatorI: LoggedNetworkNumber
    private lateinit var elevatorD: LoggedNetworkNumber
    private lateinit var elevatorV: LoggedNetworkNumber
    private lateinit var elevatorS: LoggedNetworkNumber
    private lateinit var elevatorG: LoggedNetworkNumber
    private lateinit var cruiseV: LoggedNetworkNumber
    private lateinit var acc: LoggedNetworkNumber
    private lateinit var jerk: LoggedNetworkNumber

    private val elevatorRightConfigs: TalonFXConfiguration = TalonFXConfiguration()

    /**
     * The state of the elevator
     *
     * @return [ElevatorState], the state of the elevator
     */
    var elevatorState: ElevatorState = ElevatorState.DEFAULT

    private val motionMagicVoltage: MotionMagicVoltage = MotionMagicVoltage(0.0)

    /**
     * Creates a new instance of this ElevatorSubsystem. This constructor is private since this class
     * is a Singleton. Code should use the [.getInstance] method to get the singleton
     * instance.
     */
    init {
        motionMagicVoltage.Slot = 0

        //    motionMagicVoltage.EnableFOC = true;
        //    motionMagicVoltage.FeedForward = 0;

        // TODO increase acceleration, cruise velocity, and graph it all to see
        // how to speed it up
        // reduce timeouts for automatic scoring, autoalign speed it up even more, test in autonomous
        // with 180 command
        // algae and intake prob add or remove idkk yet

        initializeLoggedNetworkPID()
    }

    // This method will be called once per scheduler run
    override fun periodic() {
        // THIS IS JUST FOR TESTING, in reality, elevator set state is based on
        // what Jayden clicks which will be displayed on leds but not necessarily = currentState
        //    elevatorSetState = currentState;
        // setElevatorPosition(this.state)

        moveElevator(elevatorState)

        // moveElevator(testPad.leftY)

        logs {
            log("Elevator/Elevator Left is at softstop forward", elevatorMotorLeft.stickyFault_ForwardSoftLimit.value)
            log("Elevator/Elevator Left is at softstop backward", elevatorMotorRight.stickyFault_ReverseSoftLimit.value)
            log(
                "Elevator/Elevator Left Position",
                elevatorMotorLeft.position.valueAsDouble,
            )
            log(
                "Elevator/Elevator Right Position",
                elevatorMotorRight.position.valueAsDouble,
            )
            log(
                "Elevator/Elevator Left Set Speed",
                elevatorMotorLeft.velocity.valueAsDouble,
            )
            log(
                "Elevator/Elevator Right Set Speed",
                elevatorMotorRight.velocity.valueAsDouble,
            )
            log(
                "Elevator/Elevator Left Acceleration",
                elevatorMotorLeft.acceleration.valueAsDouble,
            )
            log(
                "Elevator/Elevator Right Acceleration",
                elevatorMotorRight.acceleration.valueAsDouble,
            )
            log(
                "Elevator/Elevator Supply Voltage",
                elevatorMotorLeft.supplyVoltage.valueAsDouble,
            )
            log(
                "Elevator/Elevator Motor Voltage",
                elevatorMotorLeft.motorVoltage.valueAsDouble,
            )
            log("Elevator/Elevator State", elevatorState)
            log("Elevator/Elevator To Be State", elevatorToBeSetState)
            log(
                "Elevator/Elevator Stator Current",
                elevatorMotorLeft.statorCurrent.valueAsDouble,
            )
            log(
                "Elevator/Elevator Supply Current",
                elevatorMotorLeft.supplyCurrent.valueAsDouble,
            )
            log(
                "Elevator/Elevator Stall Current",
                elevatorMotorLeft.motorStallCurrent.valueAsDouble,
            )
        }
    }

    /**
     * Set elevator to a specific state
     */
    fun setElevatorToBeSetState(default: ElevatorState) {
        elevatorToBeSetState = default
    }

    /** Stops the elevator motors  */
    fun stopMotors() {
        elevatorMotorLeft.stopMotor()
        elevatorMotorRight.stopMotor()
    }

    /**
     * Get the position of the elevator motor
     *
     * @param motor "left" or "right | The motor to get the position of
     * @return double, the position of the elevator motor and -1.0 if the motor is not "left" or
     * "right"
     */
    fun getElevatorPosValue(motor: ElevatorMotor): Double =
        when (motor) {
            ElevatorMotor.LEFT -> elevatorMotorLeft.position.valueAsDouble
            ElevatorMotor.RIGHT -> elevatorMotorRight.position.valueAsDouble
        }

    val elevatorPosAvg: Double
        /**
         * Get the average position of the elevator motors
         *
         * @return double, the average position of the elevator motors
         */
        get() = (this.getElevatorPosValue(ElevatorMotor.LEFT) + this.getElevatorPosValue(ElevatorMotor.RIGHT)) / 2

    /** Soft resets the encoders on the elevator motors  */
    fun resetEncoders() {
        elevatorMotorLeft.setPosition(0.0)
        elevatorMotorRight.setPosition(0.0)
    }

    /**
     * Moves both elevator motors to the position specified by the given ElevatorState.
     *
     * @param state The target ElevatorState whose position will be used for both motors.
     */
    fun moveElevator(state: ElevatorState) {
//        elevatorMotorLeft.move(state.pos.position)
//        elevatorMotorRight.move(state.pos.position)
        elevatorMotorLeft.setControl(voltagePos.withPosition(state.pos))
        elevatorMotorRight.setControl(voltagePos.withPosition(state.pos))
    }

    fun initializeLoggedNetworkPID() {
        elevatorP = LoggedNetworkNumber("Tuning/Elevator/Elevator P", elevatorRightConfigs.Slot0.kP)
        elevatorI = LoggedNetworkNumber("Tuning/Elevator/Elevator I", elevatorRightConfigs.Slot0.kI)
        elevatorD = LoggedNetworkNumber("Tuning/Elevator/Elevator D", elevatorRightConfigs.Slot0.kD)
        elevatorV = LoggedNetworkNumber("Tuning/Elevator/Elevator V", elevatorRightConfigs.Slot0.kV)
        elevatorS = LoggedNetworkNumber("Tuning/Elevator/Elevator S", elevatorRightConfigs.Slot0.kS)
        elevatorG = LoggedNetworkNumber("Tuning/Elevator/Elevator G", elevatorRightConfigs.Slot0.kG)

        cruiseV = LoggedNetworkNumber("Tuning/Elevator/MM Cruise Velocity", ELEVATOR_MAGIC_PINGU.velocity)
        acc = LoggedNetworkNumber("Tuning/Elevator/MM Acceleration", ELEVATOR_MAGIC_PINGU.acceleration)
        jerk = LoggedNetworkNumber("Tuning/Elevator/MM Jerk", ELEVATOR_MAGIC_PINGU.jerk)
    }

    /**
     * Sets the elevator state to the appropriate algae level based on the value of elevatorToBeSetState.
     *
     * @return true if the state was set to ALGAE_LOW or ALGAE_HIGH, false otherwise.
     */
    fun setAlgaeLevel(): Boolean =
        when (elevatorToBeSetState) {
            L2 -> {
                elevatorState = ElevatorState.ALGAE_LOW
                true
            }
            L3 -> {
                elevatorState = ElevatorState.ALGAE_HIGH
                true
            }
            else -> false
        }

    val atState: Boolean
        get() = elevatorMotorLeft.position.valueAsDouble in elevatorState.pos - 0.5..elevatorState.pos + 0.5

    fun isAlgaeReadyForShoot(): Boolean {
        if (elevatorMotorLeft.position.valueAsDouble >= (ElevatorState.ALGAE_SHOOT.pos - 10.0)) {
            return true
        }
        return false
    }

    fun isAlgaeReadyForPivot(): Boolean {
        if (elevatorMotorLeft.position.valueAsDouble >= (ElevatorState.ALGAE_SHOOT.pos - 20.0)) {
            return true
        }
        return false
    }
}
