package frc.robot

import com.pathplanner.lib.commands.PathPlannerAuto
import com.pathplanner.lib.commands.PathfindingCommand
import com.pathplanner.lib.pathfinding.Pathfinding
import edu.wpi.first.wpilibj.PowerDistribution
import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj.Threads
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import frc.robot.commands.Kommand.flipPidgey
import frc.robot.subsystems.Elevator
import frc.robot.subsystems.Intake
import frc.robot.subsystems.LED
import frc.robot.subsystems.Outtake
import frc.robot.subsystems.PhotonVision
import frc.robot.subsystems.SuperStructure
import frc.robot.subsystems.Swerve
import frc.robot.utils.RobotParameters.FieldParameters.RobotPoses.addCoralPosList
import frc.robot.utils.RobotParameters.Info.logInfo
import frc.robot.utils.RobotParameters.LiveRobotValues.LOW_BATTERY_VOLTAGE
import frc.robot.utils.RobotParameters.LiveRobotValues.lowBattery
import org.littletonrobotics.junction.LogFileUtil
import org.littletonrobotics.junction.LoggedRobot
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.networktables.NT4Publisher
import org.littletonrobotics.junction.wpilog.WPILOGReader
import org.littletonrobotics.junction.wpilog.WPILOGWriter
import xyz.malefic.frc.path.LocalADStarAK
import xyz.malefic.frc.pingu.AlertPingu
import xyz.malefic.frc.pingu.Bingu

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
class Robot : LoggedRobot() {
    private var autonomousCommand: Command? = null

    private var garbageTimer: Timer? = null
    private var batteryTimer: Timer? = null

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    override fun robotInit() {
        CommandScheduler.getInstance().registerSubsystem(
            Swerve,
            SuperStructure,
            PhotonVision,
            Outtake,
            Intake,
            Elevator,
            LED,
            Bingu,
            AlertPingu,
        )

        // Set a metadata value
        logInfo()

        // Records useful but random info
        Logger.recordMetadata("Reefscape", "Logging")

        // Set the pathfinder
        Pathfinding.setPathfinder(LocalADStarAK())

        if (isReal()) {
            // Log to NetworkTables
            Logger.addDataReceiver(NT4Publisher())

            // WARNING: PowerDistribution resource leak
            // Enables power distribution logging
            PowerDistribution(1, PowerDistribution.ModuleType.kRev)
        } else {
            // Run as fast as possible
            setUseTiming(false)

            // Pull the replay log from AdvantageScope (or prompt the user)
            val logPath = LogFileUtil.findReplayLog()

            // Read replay log
            Logger.setReplaySource(WPILOGReader(logPath))

            // Save outputs to a new log
            Logger.addDataReceiver(WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")))
        }

        // Start the logger
        Logger.start()

        // Call addCoralPosList
        addCoralPosList()

        // Initialize the garbage timer
        garbageTimer = Timer()
        batteryTimer = Timer()
        garbageTimer!!.start()

        // Configure auto builder
        Swerve.configureAutoBuilder()

        // Initialize the robot container
        val unused: Any = RobotContainer

        // Schedule the warmup command
        PathfindingCommand.warmupCommand().schedule()

        CommandScheduler.getInstance().enable()
    }

    /**
     * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
     * that you want ran during disabled, autonomous, teleoperated and test.
     *
     *
     * This runs after the mode specific periodic functions, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    override fun robotPeriodic() {
        Threads.setCurrentThreadPriority(true, 99)

        CommandScheduler.getInstance().run()
        if (garbageTimer!!.advanceIfElapsed(5.0)) System.gc()

        // Checks for low battery
        if (RobotController.getBatteryVoltage() < LOW_BATTERY_VOLTAGE) {
            batteryTimer!!.start()
            if (batteryTimer!!.advanceIfElapsed(1.5)) {
                lowBattery = true
            }
        } else {
            batteryTimer!!.stop()
            lowBattery = false
        }

        Threads.setCurrentThreadPriority(false, 99)
    }

    /**
     * This autonomous runs the autonomous command selected by your [RobotContainer] class. *
     */
    override fun autonomousInit() {
        //    autonomousCommand = robotContainer.networkChooser.getSelected();
        flipPidgey()
        autonomousCommand = PathPlannerAuto("4l4autoA")
        autonomousCommand!!.schedule()
    }

    /** This function is called once when teleop mode is initialized.  */
    override fun teleopInit() {
        if (autonomousCommand != null) autonomousCommand!!.cancel()
        //    flipPidgey();
    }

    /** This function is called once when test mode is initialized.  */
    override fun testInit() {
        CommandScheduler.getInstance().cancelAll()
    }
}
