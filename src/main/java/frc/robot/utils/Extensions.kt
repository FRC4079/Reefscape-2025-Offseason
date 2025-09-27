package frc.robot.utils

import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.util.Units.inchesToMeters
import edu.wpi.first.wpilibj.XboxController
import frc.robot.utils.RobotParameters.SwerveParameters.Thresholds.X_DEADZONE
import frc.robot.utils.RobotParameters.SwerveParameters.Thresholds.Y_DEADZONE
import kotlin.math.abs

/**
 * Extension property to convert a value in inches to meters.
 *
 * @receiver Double The value in inches.
 * @return Double The value converted to meters.
 */
val Double.inchesToMeters: Double
    get() = inchesToMeters(this)

/**
 * Extension property to load the `AprilTagFieldLayout` for the given `AprilTagFields` enum value.
 *
 * @receiver AprilTagFields The enum value representing a specific AprilTag field.
 * @return AprilTagFieldLayout The loaded field layout for the specified field.
 */
val AprilTagFields.layout: AprilTagFieldLayout
    get() = AprilTagFieldLayout.loadField(this)

/**
 * Gets the position of the left stick based on the input from the controller.
 *
 * @receiver The controller.
 * @return The coordinate representing the position of the left stick. The first element is the x-coordinate, and
 * the second element is the y-coordinate.
 */
fun XboxController.leftStickPosition(): Pair<Double, Double> {
    var x = leftX
    if (abs(x) < X_DEADZONE) x = 0.0

    var y = leftY
    if (abs(y) < Y_DEADZONE) y = 0.0

    return Pair(x, y)
}
