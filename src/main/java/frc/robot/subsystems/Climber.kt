package frc.robot.subsystems

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs
import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.ctre.phoenix6.configs.MotionMagicConfigs
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.DutyCycleOut
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.controls.PositionDutyCycle
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonSRX
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.utils.RobotParameters.ClimberParameters.PIVOT_MAGIC_PINGU
import frc.robot.utils.RobotParameters.ClimberParameters.PIVOT_PINGU
import frc.robot.utils.RobotParameters.ClimberParameters.PIVOT_SOFT_LIMIT_DOWN
import frc.robot.utils.RobotParameters.ClimberParameters.PIVOT_SOFT_LIMIT_UP
import frc.robot.utils.emu.ClimberPivotState
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber
import xyz.malefic.frc.pingu.AlertPingu.add

object Climber : SubsystemBase() {
    private val pivotMotor = TalonFX(0)
    private val pivotConfigs: TalonFXConfiguration = TalonFXConfiguration()
    private val pivotSoftLimitConfig: SoftwareLimitSwitchConfigs
    private val posRequest: PositionDutyCycle
    private val velocityRequest: VelocityTorqueCurrentFOC
    private val voltageOut: VoltageOut
    private val motionMagicVoltage: MotionMagicVoltage
    private val cycleOut: DutyCycleOut
    private var motionMagicConfigs: MotionMagicConfigs
    private val voltagePos: PositionVoltage = PositionVoltage(0.0)
    private var pivotP: LoggedNetworkNumber? = null
    private var pivotI: LoggedNetworkNumber? = null
    private var pivotD: LoggedNetworkNumber? = null
    private var pivotV: LoggedNetworkNumber? = null
    private var pivotS: LoggedNetworkNumber? = null
    private var pivotG: LoggedNetworkNumber? = null
    private var cruiseV: LoggedNetworkNumber? = null
    private var acc: LoggedNetworkNumber? = null
    private var jerk: LoggedNetworkNumber? = null

    private var clampMotor: PWMTalonSRX = PWMTalonSRX(0)

    init {
        pivotConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake
        pivotConfigs.Slot0.kP = PIVOT_PINGU.p
        pivotConfigs.Slot0.kI = PIVOT_PINGU.i
        pivotConfigs.Slot0.kD = PIVOT_PINGU.d
        pivotConfigs.Slot0.kV = PIVOT_PINGU.v!!
        pivotConfigs.Slot0.kS = PIVOT_PINGU.s!!
        pivotMotor.configurator.apply(pivotConfigs)
        val pivotMotorCurrentConfig = CurrentLimitsConfigs()
        val pivotMotorRampConfig = ClosedLoopRampsConfigs()
        pivotSoftLimitConfig = SoftwareLimitSwitchConfigs()
        pivotMotorCurrentConfig.SupplyCurrentLimit = 40.79
        pivotMotorCurrentConfig.SupplyCurrentLimitEnable = true
        pivotMotor.configurator.apply(pivotMotorCurrentConfig)
        pivotMotorRampConfig.DutyCycleClosedLoopRampPeriod = 0.0
        pivotMotor.configurator.apply(pivotMotorRampConfig)
        pivotSoftLimitConfig.ForwardSoftLimitEnable = true
        pivotSoftLimitConfig.ReverseSoftLimitEnable = true
        pivotSoftLimitConfig.ForwardSoftLimitThreshold = PIVOT_SOFT_LIMIT_UP
        pivotSoftLimitConfig.ReverseSoftLimitThreshold = PIVOT_SOFT_LIMIT_DOWN
        pivotConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive
        pivotConfigs.SoftwareLimitSwitch = pivotSoftLimitConfig
        pivotConfigs.MotorOutput.DutyCycleNeutralDeadband = 0.1
        pivotMotor.configurator.apply(pivotSoftLimitConfig)
        velocityRequest = VelocityTorqueCurrentFOC(0.0)
        posRequest = PositionDutyCycle(0.0)
        voltageOut = VoltageOut(0.0)
        motionMagicVoltage = MotionMagicVoltage(0.0)
        cycleOut = DutyCycleOut(0.0)
        motionMagicConfigs = pivotConfigs.MotionMagic
        motionMagicConfigs.MotionMagicCruiseVelocity = PIVOT_MAGIC_PINGU.velocity
        motionMagicConfigs.MotionMagicAcceleration = PIVOT_MAGIC_PINGU.acceleration
        motionMagicConfigs.MotionMagicJerk = PIVOT_MAGIC_PINGU.jerk
        motionMagicVoltage.Slot = 0
        velocityRequest.OverrideCoastDurNeutral = false

        voltageOut.OverrideBrakeDurNeutral = false
        voltageOut.EnableFOC = true

        cycleOut.EnableFOC = false
        pivotMotor.setPosition(0.0)

        pivotMotor.configurator.apply(pivotConfigs)

        pivotMotor.configurator.apply(motionMagicConfigs)

        add(pivotMotor, "ClimberPivot")
        initializeLoggedNetworkPID()
    }
    fun initializeLoggedNetworkPID() {
        pivotP =
            LoggedNetworkNumber("Tuning/pivot/pivot P", pivotConfigs.Slot0.kP)
        pivotI =
            LoggedNetworkNumber("Tuning/pivot/pivot I", pivotConfigs.Slot0.kI)
        pivotD =
            LoggedNetworkNumber("Tuning/pivot/pivot D", pivotConfigs.Slot0.kD)
        pivotV =
            LoggedNetworkNumber("Tuning/pivot/pivot V", pivotConfigs.Slot0.kV)
        pivotS =
            LoggedNetworkNumber("Tuning/pivot/pivot S", pivotConfigs.Slot0.kS)
        pivotG =
            LoggedNetworkNumber("Tuning/pivot/pivot G", pivotConfigs.Slot0.kG)

        cruiseV =
            LoggedNetworkNumber(
                "Tuning/pivot/MM Cruise Velocity",
                motionMagicConfigs.MotionMagicCruiseVelocity,
            )
        acc =
            LoggedNetworkNumber(
                "Tuning/pivot/MM Acceleration",
                motionMagicConfigs.MotionMagicAcceleration,
            )
        jerk = LoggedNetworkNumber("Tuning/pivot/MM Jerk", motionMagicConfigs.MotionMagicJerk)

    }
    fun setPivotPos(state: ClimberPivotState) {
        pivotMotor.setControl(voltagePos.withPosition(state.pos))
    }
    fun closeClampMotor() {
        clampMotor.set(-1.0);
    }
    fun openClampMotor() {
        clampMotor.set(1.0);
    }
    fun speedClamp(speed: Double) {
        clampMotor.set(speed)
    }
}