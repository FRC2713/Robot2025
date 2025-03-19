package frc.robot.commands.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.SetpointConstants;
import frc.robot.commands.AlgaeClawCmds;
import frc.robot.commands.RollerCmds;

import org.littletonrobotics.junction.Logger;

/**
 * Collection of 
 */
public class EndEffector {
  public static Command CORAL_SCORE =
      Commands.sequence(
          Commands.runOnce(() -> Logger.recordOutput("Active EE", "CORAL_SCORE")),
          RollerCmds.setEnableLimitSwitch(false),
          RollerCmds.setSpeed(SetpointConstants.Roller.L2_PLUS_CORAL_SCORE_SPEED));

  public static Command ALGAE_GRAB =
      Commands.sequence(
          Commands.runOnce(() -> Logger.recordOutput("Active EE", "ALGAE_GRAB")),
          AlgaeClawCmds.setSpeed(SetpointConstants.AlgaeClaw.ALGAE_GRAB_SPEED));

  public static Command ALGAE_GRAB_AND_CORAL_SCORE =
      Commands.sequence(
          Commands.runOnce(() -> Logger.recordOutput("Active EE", "ALGAE_GRAB_AND_CORAL_SCORE")),
          ALGAE_GRAB, // run algae motors
          AlgaeClawCmds.waitUntilAlgae(2), // wait for algae sensor
          CORAL_SCORE); // score coral

  public static Command PROCESSOR_SCORE =
      Commands.sequence(
          Commands.runOnce(() -> Logger.recordOutput("Active EE", "PROCESSOR_SCORE")),
          AlgaeClawCmds.setSpeed(SetpointConstants.AlgaeClaw.PROCESSOR_SCORE_SPEED));
}
