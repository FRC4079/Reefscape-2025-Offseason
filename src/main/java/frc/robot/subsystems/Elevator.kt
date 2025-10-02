package frc.robot.subsystems

import co.touchlab.kermit.Logger
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.DutyCycleOut
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.controls.PositionDutyCycle
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.utils.RobotParameters.ControllerConstants.testPad
import frc.robot.utils.RobotParameters.ElevatorParameters.ELEVATOR_MAGIC_PINGU
import frc.robot.utils.RobotParameters.ElevatorParameters.ELEVATOR_PINGU
import frc.robot.utils.RobotParameters.ElevatorParameters.ELEVATOR_SOFT_LIMIT_DOWN
import frc.robot.utils.RobotParameters.ElevatorParameters.ELEVATOR_SOFT_LIMIT_UP
import frc.robot.utils.RobotParameters.ElevatorParameters.elevatorToBeSetState
import frc.robot.utils.RobotParameters.MotorParameters.ELEVATOR_MOTOR_LEFT_ID
import frc.robot.utils.RobotParameters.MotorParameters.ELEVATOR_MOTOR_RIGHT_ID
import frc.robot.utils.RobotParameters.SwerveParameters.Thresholds.X_DEADZONE
import frc.robot.utils.RobotParameters.SwerveParameters.Thresholds.Y_DEADZONE
import frc.robot.utils.emu.ElevatorMotor
import frc.robot.utils.emu.ElevatorState
import frc.robot.utils.emu.ElevatorState.L2
import frc.robot.utils.emu.ElevatorState.L3
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber
import xyz.malefic.frc.extension.leftStickPosition
import xyz.malefic.frc.pingu.log.LogPingu.log
import xyz.malefic.frc.pingu.log.LogPingu.logs
import xyz.malefic.frc.pingu.motor.Mongu
import xyz.malefic.frc.pingu.motor.control.position
import xyz.malefic.frc.pingu.motor.control.voltage
import xyz.malefic.frc.pingu.motor.talonfx.TalonFXConfig

/**
 * The ElevatorSubsystem class is a Singleton to control the elevator motors on the robot. The class
 * sets the motor positions, gets the motor positions, stops the motors, and toggles the soft stop
 * for the elevator motor.
 */
object Elevator : SubsystemBase() {
    private val elevatorMotorLeft =
        Mongu(TalonFX(ELEVATOR_MOTOR_LEFT_ID)) {
            this as TalonFXConfig
            pingu = ELEVATOR_PINGU
            inverted = InvertedValue.CounterClockwise_Positive
            softLimits = ELEVATOR_SOFT_LIMIT_UP to ELEVATOR_SOFT_LIMIT_DOWN
            currentLimits = null
            dutyCycleNeutralDeadband = 0.1
            motionMagicPingu = ELEVATOR_MAGIC_PINGU
        }
    private val elevatorMotorRight =
        Mongu(TalonFX(ELEVATOR_MOTOR_RIGHT_ID)) {
            this as TalonFXConfig
            pingu = ELEVATOR_PINGU
            softLimits = ELEVATOR_SOFT_LIMIT_UP to ELEVATOR_SOFT_LIMIT_DOWN
            currentLimits = null
            dutyCycleNeutralDeadband = 0.1
            motionMagicPingu = ELEVATOR_MAGIC_PINGU
        }

    private val posRequest: PositionDutyCycle = PositionDutyCycle(0.0)
    private val velocityRequest: VelocityTorqueCurrentFOC = VelocityTorqueCurrentFOC(0.0)

    private lateinit var elevatorP: LoggedNetworkNumber
    private lateinit var elevatorI: LoggedNetworkNumber
    private lateinit var elevatorD: LoggedNetworkNumber
    private lateinit var elevatorV: LoggedNetworkNumber
    private lateinit var elevatorS: LoggedNetworkNumber
    private lateinit var elevatorG: LoggedNetworkNumber
    private lateinit var cruiseV: LoggedNetworkNumber
    private lateinit var acc: LoggedNetworkNumber
    private lateinit var jerk: LoggedNetworkNumber

    private val voltageOut: VoltageOut = VoltageOut(0.0)
    private val elevatorLeftConfigs: TalonFXConfiguration = TalonFXConfiguration()
    private val elevatorRightConfigs: TalonFXConfiguration = TalonFXConfiguration()

    /**
     * The state of the elevator
     *
     * @return [ElevatorState], the state of the elevator
     */
    var state: ElevatorState = ElevatorState.DEFAULT

    private val motionMagicVoltage: MotionMagicVoltage = MotionMagicVoltage(0.0)

    private val cycleOut: DutyCycleOut = DutyCycleOut(0.0)

    /**
     * Creates a new instance of this ElevatorSubsystem. This constructor is private since this class
     * is a Singleton. Code should use the [.getInstance] method to get the singleton
     * instance.
     */
    init {

        motionMagicVoltage.Slot = 0

        //    motionMagicVoltage.EnableFOC = true;
        //    motionMagicVoltage.FeedForward = 0;

        // TODO test elevator with FOC, increase acceleration, cruise velocity, and graph it all to see
        // how to speed it up
        // reduce timeouts for automatic scoring, autoalign speed it up even more, test in autonomous
        // with 180 command
        // algae and intake prob add or remove idkk yet
        velocityRequest.OverrideCoastDurNeutral = false

        voltageOut.OverrideBrakeDurNeutral = false
        voltageOut.EnableFOC = true

        cycleOut.EnableFOC = false

        elevatorMotorLeft.set(0.0.position)
        elevatorMotorRight.set(0.0.position)

//        add(elevatorMotorLeft, "left elevator")
//        add(elevatorMotorRight, "right elevator")

        initializeLoggedNetworkPID()
    }

    // This method will be called once per scheduler run
    override fun periodic() {
//        System.out.println("periodic Elevator")
        // THIS IS JUST FOR TESTING, in reality, elevator set state is based on
        // what Jayden clicks which will be displayed on leds but not necessarily = currentState
        //    elevatorSetState = currentState;
        // setElevatorPosition(this.state)

        // setElevatorPosition(ElevatorState.L4)

        moveElevator(testPad.leftY)
        // recordOutput("Elevator/Test Pad Left Stick Position X", testPad.leftStickPosition(X_DEADZONE, Y_DEADZONE).first)
        // recordOutput("Elevator/Test Pad Left Stick Position Y", testPad.leftStickPosition(X_DEADZONE, Y_DEADZONE).second)

        logs {
            log("Elevator/Test Pad Left Stick Position X", testPad.leftStickPosition(X_DEADZONE, Y_DEADZONE).first)
            log("Elevator/Test Pad Left Stick Position Y", testPad.leftStickPosition(X_DEADZONE, Y_DEADZONE).second)
            log("Elevator/Elevator Left is at softstop forward", elevatorMotorLeft.motor.stickyFault_ForwardSoftLimit.value)
            log("Elevator/Elevator Left is at softstop backward", elevatorMotorRight.motor.stickyFault_ReverseSoftLimit.value)
            log(
                "Elevator/Elevator Left Position",
                elevatorMotorLeft.motor.position.valueAsDouble,
            )
            log(
                "Elevator/Elevator Right Position",
                elevatorMotorRight.motor.position.valueAsDouble,
            )
            log(
                "Elevator/Elevator Left Set Speed",
                elevatorMotorLeft.motor.velocity.valueAsDouble,
            )
            log(
                "Elevator/Elevator Right Set Speed",
                elevatorMotorRight.motor.velocity.valueAsDouble,
            )
            log(
                "Elevator/Elevator Left Acceleration",
                elevatorMotorLeft.motor.acceleration.valueAsDouble,
            )
            log(
                "Elevator/Elevator Right Acceleration",
                elevatorMotorRight.motor.acceleration.valueAsDouble,
            )
            log(
                "Elevator/Elevator Supply Voltage",
                elevatorMotorLeft.motor.supplyVoltage.valueAsDouble,
            )
            log(
                "Elevator/Elevator Motor Voltage",
                elevatorMotorLeft.motor.motorVoltage.valueAsDouble,
            )
            log("Elevator/Elevator State", state.toString())
            log("Elevator/Elevator To Be State", elevatorToBeSetState.toString())
            log(
                "Elevator/Elevator Stator Current",
                elevatorMotorLeft.motor.statorCurrent.valueAsDouble,
            )
            log(
                "Elevator/Elevator Supply Current",
                elevatorMotorLeft.motor.supplyCurrent.valueAsDouble,
            )
            log(
                "Elevator/Elevator Stall Current",
                elevatorMotorLeft.motor.motorStallCurrent.valueAsDouble,
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
        voltageOut.Output = -0.014
        elevatorMotorLeft.motor.setControl(voltageOut)
        elevatorMotorRight.motor.setControl(voltageOut)
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
            ElevatorMotor.LEFT -> elevatorMotorLeft.motor.position.valueAsDouble
            ElevatorMotor.RIGHT -> elevatorMotorRight.motor.position.valueAsDouble
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
        elevatorMotorLeft.set(0.0.position)
        elevatorMotorRight.set(0.0.position)
    }

//    /** Toggles the soft stop for the elevator motor  */
//    fun toggleSoftStop() {
//        ElevatorParameters.isSoftLimitEnabled = !ElevatorParameters.isSoftLimitEnabled
//        leftSoftLimitConfig.ReverseSoftLimitEnable = ElevatorParameters.isSoftLimitEnabled
//        leftSoftLimitConfig.ReverseSoftLimitThreshold = 0.0
//
//        rightSoftLimitConfig.ReverseSoftLimitEnable = ElevatorParameters.isSoftLimitEnabled
//        rightSoftLimitConfig.ReverseSoftLimitThreshold = 0.0
//
//        elevatorMotorLeft.configurator.apply(leftSoftLimitConfig)
//        elevatorMotorRight.configurator.apply(rightSoftLimitConfig)
//    }

    /**
     * Move the elevator motor at a specific velocity
     *
     * @param speed double, the velocity to move the elevator motor at
     */
    fun moveElevator(speed: Double) {
        val velocity = -speed * 15
        Logger.d { "Speed is $velocity" }
//        if (abs(velocity) >= deadband) {
//            elevatorMotorLeft.setControl(cycleOut.withOutput(velocity))
//            elevatorMotorRight.setControl(cycleOut.withOutput(velocity))
//            elevatorMotorRight.set(velocity)
//            elevatorMotorLeft.set(velocity)
//        } else {
//            stopMotors()
//        }

        elevatorMotorRight.set(velocity.voltage)
        elevatorMotorLeft.set(velocity.voltage)
    }

    fun initializeLoggedNetworkPID() {
        elevatorP =
            LoggedNetworkNumber("Tuning/Elevator/Elevator P", elevatorRightConfigs.Slot0.kP)
        elevatorI =
            LoggedNetworkNumber("Tuning/Elevator/Elevator I", elevatorRightConfigs.Slot0.kI)
        elevatorD =
            LoggedNetworkNumber("Tuning/Elevator/Elevator D", elevatorRightConfigs.Slot0.kD)
        elevatorV =
            LoggedNetworkNumber("Tuning/Elevator/Elevator V", elevatorRightConfigs.Slot0.kV)
        elevatorS =
            LoggedNetworkNumber("Tuning/Elevator/Elevator S", elevatorRightConfigs.Slot0.kS)
        elevatorG =
            LoggedNetworkNumber("Tuning/Elevator/Elevator G", elevatorRightConfigs.Slot0.kG)

        cruiseV =
            LoggedNetworkNumber(
                "Tuning/Elevator/MM Cruise Velocity",
                ELEVATOR_MAGIC_PINGU.velocity,
            )
        acc =
            LoggedNetworkNumber(
                "Tuning/Elevator/MM Acceleration",
                ELEVATOR_MAGIC_PINGU.acceleration,
            )
        jerk = LoggedNetworkNumber("Tuning/Elevator/MM Jerk", ELEVATOR_MAGIC_PINGU.jerk)
    }

    /**
     * Calibrates the elevator motor. This method calibrates the elevator motor by moving the motor up
     * until it stalls.
     */
    fun calibrateElevator() {
        while (elevatorMotorLeft.motor.motorStallCurrent.valueAsDouble < 3.0) {
            elevatorMotorLeft.motor.setControl(voltageOut.withOutput(0.5))
            elevatorMotorRight.motor.setControl(voltageOut.withOutput(0.5))
        }
    }

    /**
     * Sets the elevator state to the appropriate algae level based on the value of elevatorToBeSetState.
     *
     * @return true if the state was set to ALGAE_LOW or ALGAE_HIGH, false otherwise.
     */
    fun setAlgaeLevel(): Boolean =
        when (elevatorToBeSetState) {
            L2 -> {
                state = ElevatorState.ALGAE_LOW
                true
            }
            L3 -> {
                state = ElevatorState.ALGAE_HIGH
                true
            }
            else -> false
        }
}
