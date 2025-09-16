package frc.robot.utils.emu

import frc.robot.subsystems.Outtake

/**
 * Enum class representing the states of the coral manipulator.
 *
 * @property block The function associated with the coral state
 */
enum class OuttakeState(
    @JvmField val block: Outtake.() -> Unit,
) {
    /** Represents the state when the coral manipulator is intaking a coral piece. */
    CORAL_SHOOT({ shootCoral() }),

    /** Represents the state when the coral manipulator is holding a coral piece. */
    CORAL_HOLD({ stopMotors() }),

    /** Represents the state when the coral manipulator is releasing a coral piece. */
    CORAL_REVERSE({ reverseCoral() }),

    /** Represents the state when the coral manipulator is holding algae. */
    ALGAE_HOLD({ slowAlgaeScoreMotors() }),

    /**
     * Represents the state when the outtake mechanism is shooting algae.
     * Invokes `Outtake.shootAlgae()`.
     */
    ALGAE_SHOOT({ shootAlgae() }),

    /** Represents the state when the outtake mechanism is stowed. */
    STOWED({ stow() }),
}
