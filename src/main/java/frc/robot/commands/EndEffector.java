package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.SSConstants;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public enum EndEffector {
  CORAL_SCORE(
      () ->
          Commands.sequence(
              RollerCmds.setEnableLimitSwitch(false),
              RollerCmds.setSpeed(SSConstants.Roller.L2_PLUS_CORAL_SCORE_SPEED))),

  ALGAE_GRAB(() -> AlgaeClawCmds.setSpeed(SSConstants.AlgaeClaw.ALGAE_GRAB_SPEED)),

  ALGAE_GRAB_AND_CORAL_SCORE(
      () ->
          Commands.sequence(
              ALGAE_GRAB.getCommand(), // run algae motors
              AlgaeClawCmds.waitUntilAlgae(2), // wait for algae sensor
              CORAL_SCORE.getCommand())), // score coral

  PROCESSOR_SCORE(
      () -> Commands.sequence(AlgaeClawCmds.setSpeed(SSConstants.AlgaeClaw.PROCESSOR_SCORE_SPEED)));

  private Supplier<Command> cmd;

  private EndEffector(Supplier<Command> cmd) {
    this.cmd = cmd;
  }

  public Command getCommand() {
    return Commands.sequence(
        new InstantCommand(() -> Logger.recordOutput("Active EE", this.toString())),
        Commands.print("Starting EE: " + this.toString()),
        cmd.get());
  }

  public Command delayCommand(double seconds) {
    return Commands.sequence(Commands.waitSeconds(seconds), getCommand());
  }
}
