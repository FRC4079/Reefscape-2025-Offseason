package frc.robot.utils.emu

sealed class SwerveDriveState {
    // Top-level Auto option
    object Auto : SwerveDriveState()

    // Tele-op drive with subcategories
    object ManualDrive : SwerveDriveState()

    data class SwerveAlignment(
        val dir: Direction,
    ) : SwerveDriveState()
}
