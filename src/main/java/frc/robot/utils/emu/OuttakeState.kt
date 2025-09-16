package frc.robot.utils.emu

import frc.robot.subsystems.Intake
import frc.robot.subsystems.Outtake

/**
 * Enum class representing the states of the coral manipulator.
 *
 * @property block The function associated with the coral state
 */
enum class OuttakeState(
    @JvmField val block: () -> Unit,
) {
    /** Represents the state when the coral manipulator is intaking a coral piece. */
    CORAL_INTAKE({ Intake.startCoralIntake() }),

    /** Represents the state when the coral manipulator is holding a coral piece. */
    CORAL_HOLD({ Intake.stopMotors() }),

    /** Represents the state when the coral manipulator is releasing a coral piece. */
    CORAL_REVERSE({ Intake.reverseMotors() }),

    /** Represents the state when the coral manipulator is intaking algae. */
    ALGAE_INTAKE({ Intake.intakeAlgae() }),

    /** Represents the state when the coral manipulator is holding algae. */
    ALGAE_HOLD({ Outtake.slowAlgaeScoreMotors() }),

    STOWED({ Outtake.stow() }),
}
