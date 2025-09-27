package frc.robot.utils

import co.touchlab.kermit.Logger
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform2d
import frc.robot.utils.RobotParameters.FieldParameters.AprilTagField
import frc.robot.utils.RobotParameters.FieldParameters.X_OFFSET_FROM_TAG_FOR_SCORING_INCHES
import frc.robot.utils.RobotParameters.FieldParameters.Y_OFFSET_FROM_TAG_FOR_SCORING_ON_REEF_INCHES
import frc.robot.utils.emu.Direction

/**
 * Utility object for pose-related calculations.
 */
object Pose {
    /**
     * Calculates the desired scoring pose for the robot based on the AprilTag ID and scoring direction.
     *
     * @param tagID The ID of the AprilTag (must be in the range 1..22).
     * @param dir The direction from which the robot will score (e.g., LEFT or RIGHT).
     * @return The desired scoring [edu.wpi.first.math.geometry.Pose2d] for the robot, or [edu.wpi.first.math.geometry.Pose2d.kZero] if the tagID is invalid.
     */
    fun getDesiredScorePose(
        tagID: Int,
        dir: Direction,
    ): Pose2d {
        if (tagID !in 1..22) {
            Logger.e("Invalid tagID: $tagID") { "Tag ID must be in the range 1..22" }
            return Pose2d.kZero
        }

        val tagPose =
            AprilTagField.layout
                .getTagPose(tagID)
                .get()
                .toPose2d()
        val xOffset = X_OFFSET_FROM_TAG_FOR_SCORING_INCHES.inchesToMeters
        val ySign = if (dir == Direction.RIGHT) 1.0 else -1.0
        val yOffset = ySign * Y_OFFSET_FROM_TAG_FOR_SCORING_ON_REEF_INCHES.inchesToMeters

        return tagPose.plus(Transform2d(xOffset, yOffset, Rotation2d.kZero))
    }
}
