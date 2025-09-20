/**
 * TalonFX Configuration Extension Documentation
 * 
 * This file documents the usage of @OmyDaGreat/Malefix TalonFX extension function
 * `configureWithDefaults` used throughout the robot subsystems.
 * 
 * Updated subsystems:
 * - Outtake.kt: Already using configureWithDefaults
 * - Intake.kt: Already using configureWithDefaults  
 * - Elevator.kt: Updated to use configureWithDefaults
 * - SwerveModule.kt: Updated to use configureWithDefaults
 * 
 * The configureWithDefaults extension function simplifies TalonFX motor configuration
 * by providing sensible defaults while allowing customization through optional parameters:
 * 
 * Usage examples:
 * motor.configureWithDefaults(pingu)
 * motor.configureWithDefaults(pingu, neutralMode = NeutralModeValue.Coast)
 * motor.configureWithDefaults(pingu, inverted = InvertedValue.Clockwise_Positive)
 * motor.configureWithDefaults(pingu, currentLimits = 30.0 to 30.0)
 * motor.configureWithDefaults(pingu, neutralMode = NeutralModeValue.Brake, inverted = InvertedValue.CounterClockwise_Positive, currentLimits = 40.0 to 40.0)
 */