package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.RobotParameters;
import frc.robot.utils.RobotParameters.CoralManipulatorParameters;
import xyz.malefic.frc.pingu.AlertPingu;
import xyz.malefic.frc.pingu.VoltagePingu;

import static frc.robot.utils.RobotParameters.AlgaeManipulatorParameters.algaeIntaking;
import static frc.robot.utils.RobotParameters.ClimberParameters.CLIMBER_PINGU;
import static frc.robot.utils.RobotParameters.CoralManipulatorParameters.*;
import static frc.robot.utils.RobotParameters.MotorParameters.*;
import static frc.robot.utils.emu.CoralState.*;
import static xyz.malefic.frc.pingu.LogPingu.log;
import static xyz.malefic.frc.pingu.LogPingu.logs;

public class Climber extends SubsystemBase {
  private final TalonFX climbPivotMotor;
  private final PWMTalonSRX cageLockMotor;
  private final VoltageOut voltageOut;
  private final VoltageOut voltageOutFeeder;

  private final DigitalInput coralSensor;

  private boolean motorsRunning = false;

  /**
   * The Singleton instance of this CoralManipulatorSubsystem. Code should use the {@link
   * #getInstance()} method to get the single instance (rather than trying to construct an instance
   * of this class.)
   */
  private static final Climber INSTANCE = new Climber();

  /**
   * Returns the Singleton instance of this CoralManipulatorSubsystem. This static method should be
   * used, rather than the constructor, to get the single instance of this class. For example:
   * {@code CoralManipulatorSubsystem.getInstance();}
   */
  @SuppressWarnings("WeakerAccess")
  public static Climber getInstance() {
    return INSTANCE;
  }

  /**
   * Creates a new instance of this CoralManipulatorSubsystem. This constructor is private since
   * this class is a Singleton. Code should use the {@link #getInstance()} method to get the
   * singleton instance.
   */
  private Climber() {
    climbPivotMotor = new TalonFX(CLIMB_PIVOT_MOTOR_ID);
    cageLockMotor = new PWMTalonSRX(CAGE_LOCK_MOTOR_ID);

    coralSensor = new DigitalInput(CORAL_SENSOR_ID);

    TalonFXConfiguration climbPivotConfiguration = new TalonFXConfiguration();
//    PWNTalonSRXConfiguration cageLockConfiguration = new PWMTalonSRXConfiguration();

    climbPivotConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    climbPivotConfiguration.Slot0.kP = CLIMBER_PINGU.getP();
    climbPivotConfiguration.Slot0.kI = CLIMBER_PINGU.getI();
    climbPivotConfiguration.Slot0.kD = CLIMBER_PINGU.getD();
    climbPivotConfiguration.Slot0.kV = CLIMBER_PINGU.getV();

    climbPivotMotor.getConfigurator().apply(climbPivotConfiguration);

    CurrentLimitsConfigs climbPivotCurrentConfig = new CurrentLimitsConfigs();

    climbPivotCurrentConfig.SupplyCurrentLimit = 40;
    climbPivotCurrentConfig.SupplyCurrentLimitEnable = true;
    climbPivotCurrentConfig.StatorCurrentLimit = 40;
    climbPivotCurrentConfig.StatorCurrentLimitEnable = true;

    climbPivotMotor.getConfigurator().apply(climbPivotCurrentConfig);

    // on
    climbPivotConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    x.getConfigurator().apply(climbPivotConfiguration);

    voltageOut = new VoltageOut(0);
    voltageOutFeeder = new VoltageOut(0);

    AlertPingu.add(climbPivotMotor, "climber pivot motor");
  }

  /**
   * If the coral sensor is triggered, set the hasPiece boolean to true. (hasPiece = true,
   * sensorDetect = true), motors spinning If the manipulator has a piece, but the sensor no longer
   * detects it, stop the motors. (hasPiece = true, sensorDetect = false), motors stop If the
   * manipulator should start, but the motors are not running, start the motors (hasPiece = false,
   * sensorDetect = false), motors spinning by setting if it has a piece to false, due to the fact
   * that the manipulator should not have a piece after the motors are started again.
   *
   * <p>The manipulator motors should be on by default, as per Aaron's request.
   */
  @Override
  public void periodic() {




  }
`
  /** Stops the coral manipulator motors */
  public void stopClimbPivotMotor() {
    //    coralFeederMotor.stopMotor();
    climbPivotMotor.stopMotor();
  }

  /** Starts the coral manipulator motors */
  public void startCoralIntake() {
    voltageOut.Output = 5.0;
    coralScoreMotor.setControl(voltageOut);
    this.setHasPiece(false);
    //    isCoralIntaking = true;
  }

  /** Scores the coral motors */
  public void scoreCoral() {
    voltageOut.Output = 3.0;
    coralScoreMotor.setControl(voltageOut);
    this.motorsRunning = true;
    this.setHasPiece(false);
  }

  /** Starts the coral manipulator motors */
  public void slowCoralIntake() {
    coralScoreMotor.setControl(VoltagePingu.setOutput(4.0));
    //    coralFeederMotor.setControl(VoltagePingu.setOutput(4.0));
    this.setHasPiece(true);
  }

  /** Starts the coral manipulator motors */
  public void reverseMotors() {
    coralFeederMotor.setControl(VoltagePingu.setOutput(-4.5));
    coralScoreMotor.setControl(VoltagePingu.setOutput(-4.5));
    starFeederMotor.setControl(VoltagePingu.setOutput(-4.5));
    this.motorsRunning = true;
  }

  /**
   * Activates the algae intake process. This method stops the current motors, sets the voltage
   * output to 4.5, and starts the coral score motor to intake algae.
   */
  public void algaeIntake() {
    this.stopMotors();
    voltageOut.Output = -4.5;
    coralScoreMotor.setControl(voltageOut);
    this.motorsRunning = true;
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
    coralScoreMotor.setControl(voltageOut);
  }

  /**
   * Ejects the algae by setting the voltage output to 4.0 and controlling the coral score motor.
   */
  public void ejectAlgae() {
    voltageOut.Output = 4.0;
    coralScoreMotor.setControl(voltageOut);
  }

  /**
   * Gets the state of the coral manipulator
   *
   * @return The state of the coral manipulator
   */
  public boolean getCoralSensor() {
    return !coralSensor.get();
  }
}
