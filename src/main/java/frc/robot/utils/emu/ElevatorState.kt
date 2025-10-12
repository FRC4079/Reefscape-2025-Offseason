package frc.robot.utils.emu

/**
 * The ElevatorState enum represents the different levels that the elevator can be in within the
 * robot's system.
 *
 * @property pos The position of the elevator in rotations.
 */
enum class ElevatorState(
    var pos: Double,
) {
    /** Represents the default state of the elevator  */
    DEFAULT(0.1),

    /** Represents the first level of the elevator for coral. */
    L1(8.0),

    /** Represents the second level of the elevator for coral.  */
    L2(18.0),

    /** Represents the third level of the elevator for coral.  */
    L3(29.2),

    /** Represents the fourth level of the elevator for coral.  */
    L4(46.22),

    /** Represents the low level of the elevator for algae.  */
    ALGAE_LOW(24.0),

    /** Represents the high level of the elevator for algae.  */
    ALGAE_HIGH(26.0),

    /** Represents the barge level of the elevator for algae.  */
    ALGAE_SHOOT(50.0),
}
