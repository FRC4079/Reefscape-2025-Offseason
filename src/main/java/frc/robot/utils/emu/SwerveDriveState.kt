package frc.robot.utils.emu

/**
 * Represents the state of a swerve drive system.
 * This is a sealed class, meaning all possible subclasses are defined within this file.
 */
sealed class SwerveDriveState {
    /**
     * Represents the autonomous driving state.
     */
    object Auto : SwerveDriveState()

    /**
     * Represents the manual driving state, typically used during teleoperation.
     */
    object ManualDrive : SwerveDriveState()

    /**
     * Represents the state for automatic alignment based on a specified direction.
     *
     * @property dir The direction used for alignment.
     */
    data class SwerveAlignment(
        val dir: Direction,
    ) : SwerveDriveState()
}
