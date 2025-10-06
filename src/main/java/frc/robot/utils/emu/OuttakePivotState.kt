package frc.robot.utils.emu

/**
 * Enum class representing the state of the pivot.
 *
 * @property pos The position of the pivot in rotations.
 */
enum class OuttakePivotState(
    val pos: Double,
) {
    ALGAE_INTAKE(3.5),

    ALGAE_HOLD(0.0),

    ALGAE_SHOOT(0.0),

    CORAL_L1(0.0),

    CORAL_L23(-2.9),

    CORAL_L4(-3.0),

    INTAKE(-0.75),
}
