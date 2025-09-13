package frc.robot.utils.emu

import frc.robot.subsystems.Intake.getInstance
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
    CORAL_INTAKE({ getInstance().startCoralIntake() }),

    /** Represents the state when the coral manipulator is holding a coral piece. */
    CORAL_HOLD({ getInstance().stopMotors() }),

    /** Represents the state when the coral manipulator is releasing a coral piece. */
    CORAL_REVERSE({ getInstance().reverseMotors() }),

    /** Represents the state when the coral manipulator is intaking algae. */
    ALGAE_INTAKE({ getInstance().intakeAlgae() }),

    /** Represents the state when the coral manipulator is holding algae. */
    ALGAE_HOLD({ Outtake.getInstance().slowAlgaeScoreMotors() }),

    STOWED({ getInstance().stopMotors() }),
}
