package frc.robot.subsystems;

import static frc.robot.utils.RobotParameters.AlgaeManipulatorParameters.algaeIntaking;
import static frc.robot.utils.RobotParameters.CoralManipulatorParameters.*;
import static frc.robot.utils.RobotParameters.MotorParameters.*;
import static xyz.malefic.frc.pingu.LogPingu.*;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.emu.ElevatorState;
import xyz.malefic.frc.pingu.*;

public class Intake extends SubsystemBase {
    private final TalonFX wheelFeederMotor;
    private final TalonFX starFeederMotor;

    private final VoltageOut voltageOut;
    private final VoltageOut voltageOutFeeder;

    private boolean motorsRunning = false;

    /**
     * The Singleton instance of this IntakeSubsystem. Code should use the {@link #getInstance()}
     * method to get the single instance (rather than trying to construct an instance of this class.)
     */
    private static final Intake INSTANCE = new Intake();

    /**
     * Returns the Singleton instance of this IntakeSubsystem. This static method should be used,
     * rather than the constructor, to get the single instance of this class. For example: {@code
     * Intake.getInstance();}
     */
    @SuppressWarnings("WeakerAccess")
    public static Intake getInstance() {
        return INSTANCE;
    }

    /**
     * Creates a new instance of this IntakeSubsystem. This constructor is private since this class is
     * a Singleton. Code should use the {@link #getInstance()} method to get the singleton instance.
     */
    private Intake() {
        wheelFeederMotor = new TalonFX(CORAL_FEEDER_ID);
        starFeederMotor = new TalonFX(STAR_FEEDER_ID);

        TalonFXConfiguration coralFeederConfiguration = new TalonFXConfiguration();
        TalonFXConfiguration starFeederConfiguration = new TalonFXConfiguration();

        coralFeederConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        starFeederConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        coralFeederConfiguration.Slot0.kP = CORAL_FEEDER_PINGU.getP();
        coralFeederConfiguration.Slot0.kI = CORAL_FEEDER_PINGU.getI();
        coralFeederConfiguration.Slot0.kD = CORAL_FEEDER_PINGU.getD();
        coralFeederConfiguration.Slot0.kV = CORAL_FEEDER_PINGU.getV();

        CurrentLimitsConfigs coralFeederCurrentConfig = new CurrentLimitsConfigs();
        CurrentLimitsConfigs starFeederCurrentConfig = new CurrentLimitsConfigs();

        coralFeederCurrentConfig.SupplyCurrentLimit = 40;
        coralFeederCurrentConfig.SupplyCurrentLimitEnable = true;
        coralFeederCurrentConfig.StatorCurrentLimit = 40;
        coralFeederCurrentConfig.StatorCurrentLimitEnable = true;

        starFeederCurrentConfig.SupplyCurrentLimit = 40;
        starFeederCurrentConfig.SupplyCurrentLimitEnable = true;
        starFeederCurrentConfig.StatorCurrentLimit = 40;
        starFeederCurrentConfig.StatorCurrentLimitEnable = true;

        wheelFeederMotor.getConfigurator().apply(coralFeederCurrentConfig);
        starFeederMotor.getConfigurator().apply(starFeederCurrentConfig);

        // on
        coralFeederConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        starFeederConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        wheelFeederMotor.getConfigurator().apply(coralFeederConfiguration);
        starFeederMotor.getConfigurator().apply(starFeederConfiguration);

        voltageOut = new VoltageOut(0);
        voltageOutFeeder = new VoltageOut(0);

        AlertPingu.add(wheelFeederMotor, "Coral Feeder");
        AlertPingu.add(starFeederMotor, "Star Feeder Motor");
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
        voltageOutFeeder.Output = 5;
        wheelFeederMotor.setControl(voltageOutFeeder);
        starFeederMotor.setControl(voltageOutFeeder);

        logs(
                () -> {
                    log("Coral/Has Piece", hasPiece);
                    log("Coral/Coral Scoring", coralScoring);
                    log("Coral/motorsRunning", motorsRunning);
                    log("Coral/Coral State", outtakeState.toString());
                });

        outtakeState.block.invoke();
    }

    /**
     * Stops the coral manipulator motors
     */
    public void stopMotors() {
        wheelFeederMotor.stopMotor();
        starFeederMotor.stopMotor();
        motorsRunning = false;
    }

    /**
     * Starts the coral manipulator motors
     */
    public void startCoralIntake() {
        voltageOut.Output = 5.0;
        wheelFeederMotor.setControl(voltageOut);
        starFeederMotor.setControl(voltageOut);
        //    isCoralIntaking = true;
    }

    /**
     * Starts the coral manipulator motors
     */
    public void reverseMotors() {
        wheelFeederMotor.setControl(VoltagePingu.setOutput(-4.5));
        starFeederMotor.setControl(VoltagePingu.setOutput(-4.5));
        motorsRunning = true;
    }

    /**
     * Initiates the algae intake process. Sets the elevator to the algae level, starts the intake
     * motor, and marks the algaeIntaking flag as true.
     */
    public void intakeAlgae() {
        stopMotors();
        Elevator.setAlgaeLevel();
        setIntakeSpeed(4.5);
        algaeIntaking = true;
    }

    /**
     * Sets the speed of the algae intake motor.
     *
     * @param speed the desired speed to set for the intake motor
     */
    public void setIntakeSpeed(double speed) {
        voltageOut.Output = speed;
        wheelFeederMotor.setControl(voltageOut);
        starFeederMotor.setControl(voltageOut);
    }

    /**
     * Stops the algae intake process. Resets the elevator position to default, stops the algae intake
     * motor, and sets the algaeIntaking flag to false.
     */
    public void stopIntake() {
        Elevator.getInstance().setElevatorPosition(ElevatorState.DEFAULT);
        stopMotors();
        algaeIntaking = false;
    }
}
