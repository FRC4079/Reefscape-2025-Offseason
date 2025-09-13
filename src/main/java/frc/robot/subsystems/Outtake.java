package frc.robot.subsystems;

import static frc.robot.utils.RobotParameters.AlgaeManipulatorParameters.*;
import static frc.robot.utils.RobotParameters.CoralManipulatorParameters.*;
import static frc.robot.utils.RobotParameters.CoralManipulatorParameters.outtakeState;
import static frc.robot.utils.RobotParameters.CoralManipulatorParameters.hasPiece;
import static frc.robot.utils.RobotParameters.MotorParameters.*;
import static frc.robot.utils.emu.AlgaeCounter.*;
import static frc.robot.utils.emu.OuttakeState.*;
import static xyz.malefic.frc.pingu.LogPingu.log;
import static xyz.malefic.frc.pingu.LogPingu.logs;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.RobotParameters.*;
import frc.robot.utils.emu.AlgaePivotState;
import xyz.malefic.frc.pingu.AlertPingu;

/**
 * The Outtake class is a subsystem that interfaces with the arm system to provide control
 * over the arm motors. This subsystem is a Singleton, meaning that only one instance of this class
 * is created and shared across the entire robot code.
 */
public class Outtake extends SubsystemBase {
  /** Creates a new end effector. */
  private final TalonFX pivotMotor;
  private final TalonFX outtakeMotor;

  private final VoltageOut voltageOut;
  private final PositionVoltage voltagePos;

  private final DigitalInput coralSensor;
  private final DigitalInput algaeSensor;

  /**
   * The Singleton instance of this Outtake. Code should use the {@link #getInstance()}
   * method to get the single instance (rather than trying to construct an instance of this class.)
   */
  private static final Outtake INSTANCE = new Outtake();

  /**
   * Returns the Singleton instance of this Outtake. This static method should be used,
   * rather than the constructor, to get the single instance of this class. For example: {@code
   * armSubsystem.getInstance();}
   */
  @SuppressWarnings("WeakerAccess")
  public static Outtake getInstance() {
    return INSTANCE;
  }

  /**
   * Creates a new instance of this Outtake. This constructor is private since this class is a
   * Singleton. Code should use the {@link #getInstance()} method to get the singleton instance.
   */
  private Outtake() {
    pivotMotor = new TalonFX(ALGAE_PIVOT_MOTOR_ID);
    outtakeMotor = new TalonFX(ALGAE_INTAKE_MOTOR_ID);

    coralSensor = new DigitalInput(CoralManipulatorParameters.CORAL_SENSOR_ID);
    algaeSensor = new DigitalInput(CoralManipulatorParameters.ALGAE_SENSOR_ID);

    TalonFXConfiguration algaePivotConfiguration = new TalonFXConfiguration();
    TalonFXConfiguration algaeIntakeConfiguration = new TalonFXConfiguration();

    algaePivotConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    algaeIntakeConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    algaePivotConfiguration.Slot0.kP = AlgaeManipulatorParameters.ALGAE_PINGU.getP();
    algaePivotConfiguration.Slot0.kI = AlgaeManipulatorParameters.ALGAE_PINGU.getI();
    algaePivotConfiguration.Slot0.kD = AlgaeManipulatorParameters.ALGAE_PINGU.getD();

    algaePivotConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    algaeIntakeConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    pivotMotor.getConfigurator().apply(algaePivotConfiguration);
    outtakeMotor.getConfigurator().apply(algaeIntakeConfiguration);

    algaePivotConfiguration.CurrentLimits.SupplyCurrentLimit = 30;
    algaePivotConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
    algaePivotConfiguration.CurrentLimits.StatorCurrentLimit = 30;
    algaePivotConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;

    algaeIntakeConfiguration.CurrentLimits.SupplyCurrentLimit = 30;
    algaeIntakeConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
    algaeIntakeConfiguration.CurrentLimits.StatorCurrentLimit = 30;
    algaeIntakeConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;

    algaePivotConfiguration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0.0;
    algaePivotConfiguration.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    algaePivotConfiguration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0;
    algaePivotConfiguration.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;

    pivotMotor.getConfigurator().apply(algaePivotConfiguration);
    outtakeMotor.getConfigurator().apply(algaeIntakeConfiguration);

    //    algaeManipulatorMotorConfiguration.MotorOutput.Inverted =
    // InvertedValue.Clockwise_Positive;
    //
    //    algaeManipulatorMotorConfiguration.SoftwareLimitSwitch =
    // algaeManipulatorMotorSoftLimitConfig;

    voltageOut = new VoltageOut(0);
    voltagePos = new PositionVoltage(0);

    pivotMotor.setPosition(0);

    AlertPingu.add(pivotMotor, "algae pivot");
    AlertPingu.add(outtakeMotor, "algae intake");
  }

  // This method will be called once per scheduler run
  @Override
  public void periodic() {
    setPivotPos(algaePivotState);
    checkAlgaeSensor();
    //        setIntakeSpeed(algaePivotState);

    if (!algaeIntaking && !coralScoring) {
      if (!getCoralSensor() && !hasPiece) {
        outtakeState = CORAL_INTAKE;
      } else if (getCoralSensor() && !hasPiece) {
        // Stop the motors if the manipulator has a piece, but the sensor no longer detects it
        outtakeState = CORAL_HOLD;
        setHasPiece(true);
      } else if (!getAlgaeSensor() && hasPiece) {
        outtakeState = CORAL_HOLD;
      } else {
        outtakeState = STOWED;
      }
    }

      if (getCoralSensor()) {
          outtakeState = STOWED;
      } else {
          outtakeState = CORAL_HOLD;
      }

    logs(
        () -> {
          log("Algae/Algae Pivot Motor Position", getPivotPosValue());
          log("Algae/Algae State", algaePivotState.toString());
          log("Algae/IsAlgaeIntaking", algaeIntaking);
          log("Algae/Algae counter", algaeCounter.toString());
          log(
              "Algae/Disconnected algaeManipulatorMotor " + pivotMotor.getDeviceID(),
              pivotMotor.isConnected());
          log(
              "Algae/Algae Pivot Stator Current",
              pivotMotor.getStatorCurrent().getValueAsDouble());
          log(
              "Algae/Algae Pivot Supply Current",
              pivotMotor.getSupplyCurrent().getValueAsDouble());
          log(
              "Algae/Algae Pivot Stall Current",
              pivotMotor.getMotorStallCurrent().getValueAsDouble());
        });
  }

  /**
   * Sets the pivot state *
   *
   * @param state the state to set the algae pivot
   */
  public void setPivotPos(AlgaePivotState state) {
    pivotMotor.setControl(voltagePos.withPosition(state.pos));
  }

  /**
   * Get the position of the end effector motor
   *
   * @return double, the position of the end effector motor
   */
  public double getPivotPosValue() {
    return pivotMotor.getPosition().getValueAsDouble();
  }

  /**
   * Sets the speed of the algae outtake motor.
   *
   * @param speed the desired speed to set for the intake motor
   */
  public void setOuttakeSpeed(double speed) {
    voltageOut.Output = speed;
    outtakeMotor.setControl(voltageOut);
  }

  /** Stops the algae intake motor. */
  private void stopAlgaeMotor() {
    outtakeMotor.stopMotor();
  }

  /**
   * Checks the state of the algae sensor. Sets the algaeCounter to HOLDING if the sensor is
   * triggered, otherwise sets it to DEFAULT.
   */
  public void checkAlgaeSensor() {
    if (algaeSensor.get()) {
      algaeCounter = HOLDING;
    } else {
      algaeCounter = DEFAULT;
    }
  }

  public void shootAlgae() {
    setOuttakeSpeed(-30.0);
  }

  /**
   * Sets the state of whether the manipulator has a piece.
   *
   * @param hasPiece true if the manipulator has a piece, false otherwise
   */
  public void setHasPiece(boolean hasPiece) {
    CoralManipulatorParameters.hasPiece = hasPiece;
  }

    /**
     * Sets the voltage output to -3.0 and controls the coral score motor to slow down the algae
     * scoring process.
     */
    public void slowAlgaeScoreMotors() {
        voltageOut.Output = -3.0;
        outtakeMotor.setControl(voltageOut);
    }

  /**
   * Gets the state of the coral sensor
   *
   * @return The state of the coral manipulator
   */
  public boolean getCoralSensor() {
    return !coralSensor.get();
  }

  /**
   * Gets the state of the algae sensor
   *
   * @return The state of the algae sensor
   */
  public boolean getAlgaeSensor() {
    return !algaeSensor.get();
  }
}
