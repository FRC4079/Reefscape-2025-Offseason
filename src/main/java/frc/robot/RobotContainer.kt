package frc.robot;

import static frc.robot.commands.Kommand.*;
import static frc.robot.commands.sequencing.Sequences.*;
import static frc.robot.utils.emu.Button.*;
import static frc.robot.utils.emu.Direction.*;
import static frc.robot.utils.emu.ElevatorState.*;
import static frc.robot.utils.pingu.Bingu.*;
import static frc.robot.subsystems.SuperStructure.*;
import static frc.robot.utils.emu.State.*;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.*;
import frc.robot.commands.*;
import frc.robot.commands.sequencing.*;
import frc.robot.subsystems.*;
import frc.robot.utils.emu.ElevatorState;
import frc.robot.utils.emu.State;
import frc.robot.utils.pingu.*;

public class RobotContainer {
  public final SendableChooser<Command> networkChooser;
  public final XboxController aacrn;
  public final XboxController calamityCow;

  public RobotContainer() {
    aacrn = new XboxController(0);
    calamityCow = new XboxController(1);

    Elevator.getInstance().setDefaultCommand(padElevator(aacrn, calamityCow));
    Swerve.getInstance().setDefaultCommand(drive(aacrn));

    new CommandPingu()
            .bind("ScoreL4Left", fullScoreAuto(LEFT))
            .bind("ScoreL4Right", fullScoreAuto(RIGHT))
            .bind("HasPieceFalse", hasPieceFalse())
            .bind("MoveElevatorL4Auto", moveElevatorState(L4))
            .bind("MoveElevatorDefaultAuto", moveElevatorState(DEFAULT))
            .bind("SetL1", setElevatorState(L1))
            .bind("SetL2", setElevatorState(L2))
            .bind("SetL3", setElevatorState(L3))
            .bind("SetL4", setElevatorState(L4))
            .bind("MoveElevatorDown", setElevatorState(DEFAULT));

    networkChooser = AutoBuilder.buildAutoChooser();

    configureBindings();
  }

  private void configureBindings() {
    bindings(aacrn,
            // Alignment
            bind(LEFT_BUMPER, () -> new InstantCommand(() -> SuperStructure.queueState(new State.ScoreAlign(LEFT)))),
            bind(RIGHT_BUMPER, () -> new InstantCommand(() -> SuperStructure.queueState(new State.ScoreAlign(RIGHT)))),
            //Elevator States
            bind(Y, () -> new InstantCommand(() -> Elevator.setElvatorToBeSetState(ElevatorState.L4))),
            bind(X, () -> new InstantCommand(() -> Elevator.setElvatorToBeSetState(ElevatorState.L3))),
            bind(B, () -> new InstantCommand(() -> Elevator.setElvatorToBeSetState(ElevatorState.L2))),
            bind(DPAD_UP, () -> new InstantCommand(() -> Elevator.setElvatorToBeSetState(ElevatorState.L1))),

            //Stow Elevator
            bind(A, () -> new InstantCommand(() -> Elevator.setState(DEFAULT)))

            //Intake

            //Algae
            // left trigger
    );



    bindings(calamityCow, bind(A, Kommand::offVision));
    bindings(calamityCow, bind(B, Kommand::onVision));
  }
}
