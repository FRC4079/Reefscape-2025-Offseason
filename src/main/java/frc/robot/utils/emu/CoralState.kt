package frc.robot.utils.emu

import frc.robot.subsystems.Coral.getInstance

/**
 * Enum class representing the states of the coral manipulator.
 *
 * @property block The function associated with the coral state
 */
enum class CoralState(
    @JvmField val block: () -> Unit,
) {
    /** Represents the state when the coral manipulator is intaking a coral piece. */
    CORAL_INTAKE({ getInstance().startCoralIntake() }),

    /** Represents the state when the coral manipulator is holding a coral piece. */
    CORAL_HOLD({ getInstance().stopMotors() }),

    /** Represents the state when the coral manipulator is slowing down to hold a coral piece. */
    CORAL_SLOW({ getInstance().slowCoralIntake() }),

    /** Represents the state when the coral manipulator is releasing a coral piece. */
    CORAL_RELEASE({ getInstance().scoreCoral() }),

    /** Represents the state when the coral manipulator is intaking algae. */
    ALGAE_INTAKE({ getInstance().algaeIntake() }),

    /** Represents the state when the coral manipulator is holding algae. */
    ALGAE_HOLD({ getInstance().slowAlgaeScoreMotors() }),

    /** Represents the state when the coral manipulator is releasing algae. */
    ALGAE_RELEASE({ getInstance().ejectAlgae() }),
}
