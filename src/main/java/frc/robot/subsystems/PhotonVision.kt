package frc.robot.subsystems

import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.net.PortForwarder
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.utils.RobotParameters.PhotonVisionConstants.CAMERA_ONE_HEIGHT_METER
import frc.robot.utils.getDecentResultPairs
import frc.robot.utils.hasTargets
import org.photonvision.PhotonCamera
import org.photonvision.targeting.PhotonPipelineResult
import xyz.malefic.frc.extension.hasTargets
import xyz.malefic.frc.pingu.LogPingu.log
import xyz.malefic.frc.pingu.LogPingu.logs
import xyz.malefic.frc.sub.PhotonModule

/**
 * The PhotonVision class is a subsystem that interfaces with multiple PhotonVision cameras to
 * provide vision tracking and pose estimation capabilities. This subsystem is a Singleton that
 * manages multiple CameraModules and selects the best result based on pose ambiguity.
 *
 *
 * This subsystem provides methods to get the estimated global pose of the robot, the distance to
 * targets, and the yaw of detected AprilTags. It also provides methods to check if a tag is visible
 * and get the pivot position based on distance calculations.
 */
object PhotonVision : SubsystemBase() {
    private val cameras: MutableList<PhotonModule> = ArrayList<PhotonModule>()

    /**
     * Gets the current yaw angle to the target.
     *
     * @return The yaw angle in degrees
     */
    var yaw: Double = 0.0
        private set

    /**
     * Gets the current Y position of the target.
     *
     * @return The Y position in meters
     */
    var y: Double = 0.0
        private set

    /**
     * Gets the current distance to the target.
     *
     * @return The distance in meters
     */
    var dist: Double = 0.0
        private set
    var bestTargetID: Int = 0
        private set
    private var logCount = 0
    var resultPairs: MutableList<Pair<PhotonModule?, PhotonPipelineResult?>>?
        private set

    /**
     * Creates a new instance of this PhotonVision subsystem. This constructor is private since this
     * class is a Singleton. Code should use the [.getInstance] method to get the singleton
     * instance.
     */
    // x = 9.5
    // y = 12
    init {
        cameras.add(
            PhotonModule(
                "RightCamera",
                Transform3d(
                    Translation3d(0.27305, -0.2985, CAMERA_ONE_HEIGHT_METER),
                    Rotation3d(0.0, Math.toRadians(-25.0), Math.toRadians(45.0))
                ),
                AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded)
            )
        )
        // well calibrated camera is left camera
        cameras.add(
            PhotonModule(
                "LeftCamera",
                Transform3d(
                    Translation3d(0.27305, 0.2985, CAMERA_ONE_HEIGHT_METER),
                    Rotation3d(0.0, Math.toRadians(-25.0), Math.toRadians(-45.0))
                ),
                AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded)
            )
        )

        this.resultPairs = ArrayList<Pair<PhotonModule?, PhotonPipelineResult?>>()

        PortForwarder.add(5800, "photonvision.local", 5800)
    }

    /**
     * This method is called periodically by the CommandScheduler. It updates the tracked targets,
     * selects the best camera based on pose ambiguity, and updates logged information.
     */
    override fun periodic() {
        this.resultPairs = cameras.getDecentResultPairs() as MutableList<Pair<PhotonModule?, PhotonPipelineResult?>>?

        logs(
            Runnable {
                log("Photonvision/Does any camera exist", cameras.get(0) != null)
                log("Photonvision/Does any result pair exist", this.resultPairs != null)
                log("Photonvision/Has tag", hasTag())
                log("Photonvision/resultCamera List length", resultPairs!!.size)
                if (this.resultPairs != null) {
                    log("Photonvision/Result pairs have targets", resultPairs.hasTargets())
                }
            })

        if (this.resultPairs != null) {
            logs("Photonvision/Best Target list is empty", resultPairs!!.isEmpty())

            if (!resultPairs!!.isEmpty()) {
                logCount++
                logs("Photonvision/Best Target updated counter", logCount)
                val bestTarget = resultPairs!!.get(0).second!!.getBestTarget()
                logs("Photonvision/Best Target is not null", bestTarget != null)

                if (bestTarget != null) {
                    yaw = bestTarget.getYaw()
                    y = bestTarget.getBestCameraToTarget().getX()
                    dist = bestTarget.getBestCameraToTarget().getZ()
                    this.bestTargetID = bestTarget.getFiducialId()
                }

                logs("Photonvision/Best Target Yaw", yaw)
                logs("Photonvision/Best Target Y", y)
                logs("Photonvision/Best Target Dist", dist)
                logs("Photonvision/Best Target ID", this.bestTargetID)
            }

            logStdDev()
        }
    }

    /**
     * Checks if there is a visible tag.
     *
     * @return true if there is a visible tag and the current result pair is not null
     */
    fun hasTag(): Boolean {
        logs("Photonvision/currentResultPair not null", this.resultPairs != null)

        if (this.resultPairs != null) {
            logs("Photonvision/hasTargets currentResultPair", resultPairs.hasTargets())
        }

        return this.resultPairs != null && resultPairs.hasTargets()
    }

    fun requestCamera(cameraName: String?): PhotonCamera? {
        for (camera in cameras) {
            if (camera.cameraName == cameraName) {
                return camera.camera
            }
        }
        return null
    }

    fun fetchYaw(camera: PhotonCamera?): Double {
        for (pair in this.resultPairs!!) {
            if (pair.first!!.camera == camera) {
                return pair.second!!.getBestTarget().getYaw()
            }
        }
        return 0.0
    }

    fun fetchDist(camera: PhotonCamera?): Double {
        for (pair in this.resultPairs!!) {
            if (pair.first!!.camera == camera) {
                return pair.second!!.getBestTarget().getBestCameraToTarget().getX()
            }
        }
        return 0.0
    }

    fun fetchY(camera: PhotonCamera?): Double {
        for (pair in this.resultPairs!!) {
            if (pair.first!!.camera == camera) {
                return pair.second!!.getBestTarget().getBestCameraToTarget().getY()
            }
        }
        return 7157.0
    }

    /**
     * Logs the standard deviation norm for each camera. This method filters out cameras with null
     * standard deviations and logs the normF value of the standard deviations for each camera.
     */
    fun logStdDev() {
        cameras.stream()
            .filter { camera: PhotonModule? -> camera!!.currentStdDevs != null }
            .forEach { camera: PhotonModule? ->
                logs(
                    "Photonvision/Camera %s Std Dev NormF".toString(),
                    camera?.currentStdDevs!!.normF()
                )
            }
    }
}
