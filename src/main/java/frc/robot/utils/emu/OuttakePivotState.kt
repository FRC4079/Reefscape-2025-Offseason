package frc.robot.utils.emu

/**
 * Enum class representing the state of the pivot.
 *
 * @property pos The position of the pivot in rotations.
 */
enum class OuttakePivotState(
    val pos: Double,
) {
    /** Represents the position of the pivot for intaking the coral  */
    CORAL_INTAKE(-2.0),

    /** Represents the position of the pivot for the level one of coral  */
    CORAL_L1(0.0),

    /** Represents the position of the pivot for the level two and level three of coral  */
    CORAL_L23(-3.05),

    /** Represents the position of the pivot for the level four of coral  */
    CORAL_L4(-3.0),

    /** Represents the position of the pivot for intaking the algae  */
    ALGAE_INTAKE(-4.5),

    /** Represents the position of the pivot for holding the algae  */
    ALGAE_HOLD(0.0),

    /** Represents the position of the pivot for shooting the algae  */
    ALGAE_SHOOT(-2.0),

    /** Represents the stowed position of the pivot  */
    STOW(0.0),
}
