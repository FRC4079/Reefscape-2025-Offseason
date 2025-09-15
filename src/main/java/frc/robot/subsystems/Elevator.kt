package frc.robot.subsystems

import com.ctre.phoenix6.configs.*
import com.ctre.phoenix6.controls.*
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.utils.RobotParameters.ElevatorParameters
import frc.robot.utils.RobotParameters.ElevatorParameters.ELEVATOR_MAGIC_PINGU
import frc.robot.utils.RobotParameters.ElevatorParameters.ELEVATOR_PINGU
import frc.robot.utils.RobotParameters.ElevatorParameters.ELEVATOR_SOFT_LIMIT_DOWN
import frc.robot.utils.RobotParameters.ElevatorParameters.ELEVATOR_SOFT_LIMIT_UP
import frc.robot.utils.RobotParameters.ElevatorParameters.elevatorToBeSetState
import frc.robot.utils.RobotParameters.MotorParameters.ELEVATOR_MOTOR_LEFT_ID
import frc.robot.utils.RobotParameters.MotorParameters.ELEVATOR_MOTOR_RIGHT_ID
import frc.robot.utils.emu.ElevatorMotor
import frc.robot.utils.emu.ElevatorState
import frc.robot.utils.setPingu
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber
import xyz.malefic.frc.pingu.AlertPingu.add
import xyz.malefic.frc.pingu.LogPingu.log
import xyz.malefic.frc.pingu.LogPingu.logs
import kotlin.math.abs

/**
 * The ElevatorSubsystem class is a Singleton to control the elevator motors on the robot. The class
 * sets the motor positions, gets the motor positions, stops the motors, and toggles the soft stop
 * for the elevator motor.
 */
object Elevator : SubsystemBase() {
    private val elevatorMotorLeft = TalonFX(ELEVATOR_MOTOR_LEFT_ID)
    private val elevatorMotorRight = TalonFX(ELEVATOR_MOTOR_RIGHT_ID)

    private val posRequest: PositionDutyCycle
    private val velocityRequest: VelocityTorqueCurrentFOC

    private val leftSoftLimitConfig: SoftwareLimitSwitchConfigs
    private val rightSoftLimitConfig: SoftwareLimitSwitchConfigs

    private var elevatorP: LoggedNetworkNumber? = null
    private var elevatorI: LoggedNetworkNumber? = null
    private var elevatorD: LoggedNetworkNumber? = null
    private var elevatorV: LoggedNetworkNumber? = null
    private var elevatorS: LoggedNetworkNumber? = null
    private var elevatorG: LoggedNetworkNumber? = null
    private var cruiseV: LoggedNetworkNumber? = null
    private var acc: LoggedNetworkNumber? = null
    private var jerk: LoggedNetworkNumber? = null

    private val voltageOut: VoltageOut
    private val elevatorLeftConfigs: TalonFXConfiguration
    private val elevatorRightConfigs: TalonFXConfiguration

    private var motionMagicConfigs: MotionMagicConfigs

    /**
     * Get the state of the elevator motor
     *
     * @return ElevatorState, the state of the elevator motor
     */
    /** Sets the elevator state  */
    var state: ElevatorState = ElevatorState.DEFAULT

    private val motionMagicVoltage: MotionMagicVoltage

    private val cycleOut: DutyCycleOut

    /**
     * Creates a new instance of this ElevatorSubsystem. This constructor is private since this class
     * is a Singleton. Code should use the [.getInstance] method to get the singleton
     * instance.
     */
    init {

        val elevatorLeftConfigurator = elevatorMotorLeft.configurator
        val elevatorRightConfigurator = elevatorMotorRight.configurator

        elevatorLeftConfigs = TalonFXConfiguration()
        elevatorRightConfigs = TalonFXConfiguration()

        elevatorLeftConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake
        elevatorRightConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake

        elevatorLeftConfigs.Slot0.kP = ELEVATOR_PINGU.p
        elevatorLeftConfigs.Slot0.kI = ELEVATOR_PINGU.i
        elevatorLeftConfigs.Slot0.kD = ELEVATOR_PINGU.d
        elevatorLeftConfigs.Slot0.kV = ELEVATOR_PINGU.v!!
        elevatorLeftConfigs.Slot0.kS = ELEVATOR_PINGU.s!!
        elevatorLeftConfigs.Slot0.kG = ELEVATOR_PINGU.g!!

        elevatorRightConfigs.Slot0.kP = ELEVATOR_PINGU.p
        elevatorRightConfigs.Slot0.kI = ELEVATOR_PINGU.i
        elevatorRightConfigs.Slot0.kD = ELEVATOR_PINGU.d
        elevatorRightConfigs.Slot0.kV = ELEVATOR_PINGU.v!!
        elevatorRightConfigs.Slot0.kS = ELEVATOR_PINGU.s!!
        elevatorRightConfigs.Slot0.kG = ELEVATOR_PINGU.g!!

        elevatorMotorLeft.configurator.apply(elevatorLeftConfigs)
        elevatorMotorRight.configurator.apply(elevatorRightConfigs)

        val leftMotorCurrentConfig = CurrentLimitsConfigs()
        val rightMotorCurrentConfig = CurrentLimitsConfigs()

        val leftMotorRampConfig = ClosedLoopRampsConfigs()
        val rightMotorRampConfig = ClosedLoopRampsConfigs()

        leftSoftLimitConfig = SoftwareLimitSwitchConfigs()
        rightSoftLimitConfig = SoftwareLimitSwitchConfigs()

        leftMotorCurrentConfig.SupplyCurrentLimit = 40.79
        leftMotorCurrentConfig.SupplyCurrentLimitEnable = true
        leftMotorCurrentConfig.StatorCurrentLimit = 40.79
        leftMotorCurrentConfig.StatorCurrentLimitEnable = true

        rightMotorCurrentConfig.SupplyCurrentLimit = 40.79
        rightMotorCurrentConfig.SupplyCurrentLimitEnable = true
        rightMotorCurrentConfig.StatorCurrentLimit = 40.79
        rightMotorCurrentConfig.StatorCurrentLimitEnable = true

        elevatorMotorLeft.configurator.apply(leftMotorCurrentConfig)
        elevatorMotorRight.configurator.apply(rightMotorCurrentConfig)

        leftMotorRampConfig.DutyCycleClosedLoopRampPeriod = 0.0
        rightMotorRampConfig.DutyCycleClosedLoopRampPeriod = 0.0

        elevatorMotorLeft.configurator.apply(leftMotorRampConfig)
        elevatorMotorRight.configurator.apply(rightMotorRampConfig)

        // on
        leftSoftLimitConfig.ForwardSoftLimitEnable = true
        leftSoftLimitConfig.ReverseSoftLimitEnable = true
        leftSoftLimitConfig.ForwardSoftLimitThreshold = ELEVATOR_SOFT_LIMIT_UP
        leftSoftLimitConfig.ReverseSoftLimitThreshold = ELEVATOR_SOFT_LIMIT_DOWN

        rightSoftLimitConfig.ForwardSoftLimitEnable = true
        rightSoftLimitConfig.ReverseSoftLimitEnable = true
        rightSoftLimitConfig.ReverseSoftLimitThreshold = ELEVATOR_SOFT_LIMIT_DOWN
        rightSoftLimitConfig.ForwardSoftLimitThreshold = ELEVATOR_SOFT_LIMIT_UP

        elevatorLeftConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive
        elevatorRightConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive
        elevatorLeftConfigs.SoftwareLimitSwitch = leftSoftLimitConfig
        elevatorRightConfigs.SoftwareLimitSwitch = rightSoftLimitConfig

        elevatorLeftConfigs.MotorOutput.DutyCycleNeutralDeadband = 0.1
        elevatorRightConfigs.MotorOutput.DutyCycleNeutralDeadband = 0.1

        elevatorMotorLeft.configurator.apply(leftSoftLimitConfig)
        elevatorMotorRight.configurator.apply(rightSoftLimitConfig)

        velocityRequest = VelocityTorqueCurrentFOC(0.0)
        posRequest = PositionDutyCycle(0.0)
        voltageOut = VoltageOut(0.0)
        motionMagicVoltage = MotionMagicVoltage(0.0)
        cycleOut = DutyCycleOut(0.0)

        motionMagicConfigs = elevatorLeftConfigs.MotionMagic
        motionMagicConfigs.MotionMagicCruiseVelocity = ELEVATOR_MAGIC_PINGU.velocity
        motionMagicConfigs.MotionMagicAcceleration = ELEVATOR_MAGIC_PINGU.acceleration
        motionMagicConfigs.MotionMagicJerk = ELEVATOR_MAGIC_PINGU.jerk

        motionMagicVoltage.Slot = 0

        //    motionMagicVoltage.EnableFOC = true;
        //    motionMagicVoltage.FeedForward = 0;

        // TODO test elevator with FOC, increase accleration, cruise velocity, and graph it all to see
        // how to speed it up
        // reduce timeouts for automatic scoring, autoalign speed it up even more, test in autonomous
        // with 180 command
        // algae and intake prob add or remove idkk yet
        velocityRequest.OverrideCoastDurNeutral = false

        voltageOut.OverrideBrakeDurNeutral = false
        voltageOut.EnableFOC = true

        cycleOut.EnableFOC = false

        elevatorMotorLeft.setPosition(0.0)
        elevatorMotorRight.setPosition(0.0)

        elevatorLeftConfigurator.apply(elevatorLeftConfigs)
        elevatorRightConfigurator.apply(elevatorRightConfigs)

        elevatorLeftConfigurator.apply(motionMagicConfigs)
        elevatorRightConfigurator.apply(motionMagicConfigs)

        add(elevatorMotorLeft, "left elevator")
        add(elevatorMotorRight, "right elevator")

        initializeLoggedNetworkPID()
    }

    // This method will be called once per scheduler run
    override fun periodic() {
        // THIS IS JUST FOR TESTING, in reality, elevator set state is based on
        // what Jayden clicks which will be displayed on leds but not necessarily = currenState
        //    elevatorSetState = currentState;
        setElevatorPosition(this.state)

        logs(
            Runnable {
                log(
                    "Elevator/Elevator Left Position",
                    elevatorMotorLeft.position.valueAsDouble
                )
                log(
                    "Elevator/Elevator Right Position",
                    elevatorMotorRight.position.valueAsDouble
                )
                log(
                    "Elevator/Elevator Left Set Speed",
                    elevatorMotorLeft.velocity.valueAsDouble
                )
                log(
                    "Elevator/Elevator Right Set Speed",
                    elevatorMotorRight.velocity.valueAsDouble
                )
                log(
                    "Elevator/Elevator Left Acceleration",
                    elevatorMotorLeft.acceleration.valueAsDouble
                )
                log(
                    "Elevator/Elevator Right Acceleration",
                    elevatorMotorRight.acceleration.valueAsDouble
                )
                log(
                    "Elevator/Elevator Supply Voltage",
                    elevatorMotorLeft.supplyVoltage.valueAsDouble
                )
                log(
                    "Elevator/Elevator Motor Voltage",
                    elevatorMotorLeft.motorVoltage.valueAsDouble
                )
                log("Elevator/Elevator State", state.toString())
                log("Elevator/Elevator To Be State", elevatorToBeSetState.toString())
                log(
                    "Elevator/Elevator Stator Current",
                    elevatorMotorLeft.statorCurrent.valueAsDouble
                )
                log(
                    "Elevator/Elevator Supply Current",
                    elevatorMotorLeft.supplyCurrent.valueAsDouble
                )
                log(
                    "Elevator/Elevator Stall Current",
                    elevatorMotorLeft.motorStallCurrent.valueAsDouble
                )
            })
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
        elevatorMotorLeft.setControl(voltageOut)
        elevatorMotorRight.setControl(voltageOut)
    }

    /**
     * Get the position of the elevator motor
     *
     * @param motor "left" or "right | The motor to get the position of
     * @return double, the position of the elevator motor and -1.0 if the motor is not "left" or
     * "right"
     */
    fun getElevatorPosValue(motor: ElevatorMotor): Double {
        return when (motor) {
            ElevatorMotor.LEFT -> elevatorMotorLeft.position.valueAsDouble
            ElevatorMotor.RIGHT -> elevatorMotorRight.position.valueAsDouble
        }
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

    /** Toggles the soft stop for the elevator motor  */
    fun toggleSoftStop() {
        ElevatorParameters.isSoftLimitEnabled = !ElevatorParameters.isSoftLimitEnabled
        leftSoftLimitConfig.ReverseSoftLimitEnable = ElevatorParameters.isSoftLimitEnabled
        leftSoftLimitConfig.ReverseSoftLimitThreshold = 0.0

        rightSoftLimitConfig.ReverseSoftLimitEnable = ElevatorParameters.isSoftLimitEnabled
        rightSoftLimitConfig.ReverseSoftLimitThreshold = 0.0

        elevatorMotorLeft.configurator.apply(leftSoftLimitConfig)
        elevatorMotorRight.configurator.apply(rightSoftLimitConfig)
    }

    /**
     * Move the elevator motor at a specific velocity
     *
     * @param speed double, the velocity to move the elevator motor at
     */
    fun moveElevator(speed: Double) {
        val deadband = 0.001
        val velocity = -speed * 0.3309
        if (abs(velocity) >= deadband) {
            elevatorMotorLeft.setControl(cycleOut.withOutput(velocity))
            elevatorMotorRight.setControl(cycleOut.withOutput(velocity))
        } else {
            stopMotors()
        }
    }

    /**
     * Sets the elevator motor to a specific position
     *
     * @param state the [ElevatorState] to set the elevator motor to
     */
    fun setElevatorPosition(state: ElevatorState) {
        elevatorMotorLeft.setControl(motionMagicVoltage.withPosition(state.pos))
        elevatorMotorRight.setControl(motionMagicVoltage.withPosition(state.pos))
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
                "Tuning/Elevator/MM Cruise Velocity", motionMagicConfigs.MotionMagicCruiseVelocity
            )
        acc =
            LoggedNetworkNumber(
                "Tuning/Elevator/MM Acceleration", motionMagicConfigs.MotionMagicAcceleration
            )
        jerk = LoggedNetworkNumber("Tuning/Elevator/MM Jerk", motionMagicConfigs.MotionMagicJerk)
    }

    /**
     * Updates the PID values for the elevator motors. This method sets the PID values for the
     * elevator motors and updates the Motion Magic configurations.
     */
    fun updateElevatorPID() {
        ELEVATOR_PINGU.setP(elevatorP!!)
        ELEVATOR_PINGU.setI(elevatorI!!)
        ELEVATOR_PINGU.setD(elevatorD!!)
        ELEVATOR_PINGU.setV(elevatorV!!)
        ELEVATOR_PINGU.setS(elevatorS!!)
        ELEVATOR_PINGU.setG(elevatorG!!)

        ELEVATOR_MAGIC_PINGU.setVelocity(cruiseV!!)
        ELEVATOR_MAGIC_PINGU.setAcceleration(acc!!)
        ELEVATOR_MAGIC_PINGU.setJerk(jerk!!)

        applyElevatorPIDValues()
    }

    /**
     * Applies the PID values to the elevator motors. This method sets the PID values for both the
     * left and right elevator motor configurations and applies the Motion Magic configurations.
     */
    fun applyElevatorPIDValues() {
        // Set the PID values for the left elevator motor configuration
        elevatorLeftConfigs.setPingu(ELEVATOR_PINGU)

        // Set the PID values for the right elevator motor configuration
        elevatorRightConfigs.setPingu(ELEVATOR_PINGU)

        // Update the Motion Magic configurations with the current PID values
        motionMagicConfigs = elevatorLeftConfigs.MotionMagic
        motionMagicConfigs.MotionMagicCruiseVelocity = cruiseV!!.get()
        motionMagicConfigs.MotionMagicAcceleration = acc!!.get()
        motionMagicConfigs.MotionMagicJerk = jerk!!.get()

        // Apply the updated configurations to the left elevator motor
        elevatorMotorLeft.configurator.apply(elevatorLeftConfigs)
        elevatorMotorRight.configurator.apply(elevatorRightConfigs)

        // Apply the Motion Magic configurations to the left and right elevator motors
        elevatorMotorLeft.configurator.apply(motionMagicConfigs)
        elevatorMotorRight.configurator.apply(motionMagicConfigs)
    }

    /**
     * Calibrates the elevator motor. This method calibrates the elevator motor by moving the motor up
     * until it stalls.
     */
    fun calibrateElevator() {
        while (elevatorMotorLeft.motorStallCurrent.valueAsDouble < 3.0) {
            elevatorMotorLeft.setControl(voltageOut.withOutput(0.5))
            elevatorMotorRight.setControl(voltageOut.withOutput(0.5))
        }
    }
        @JvmStatic
        fun setAlgaeLevel() {
            if (elevatorToBeSetState == ElevatorState.L2) {
                Elevator.state = ElevatorState.ALGAE_LOW
            } else if (elevatorToBeSetState == ElevatorState.L3) {
                Elevator.state = ElevatorState.ALGAE_HIGH
            }
        }
}

