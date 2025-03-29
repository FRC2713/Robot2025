package frc.robot.commands.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.SetpointConstants;
import frc.robot.commands.AlgaeClawCmds;
import frc.robot.commands.RollerCmds;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Collection of commands that ONLY have to do with the end effector. Note that the SuperStructure
 * class commands might also control end effector motors.
 */
public class EndEffector {
  public static Supplier<Command> CORAL_SCORE =
      () ->
          Commands.sequence(
              Commands.runOnce(() -> Logger.recordOutput("Active EE", "CORAL_SCORE")),
              RollerCmds.setEnableLimitSwitch(false),
              RollerCmds.setSpeed(SetpointConstants.Roller.L2_PLUS_CORAL_SCORE_SPEED));

  public static Supplier<Command> ALGAE_GRAB =
      () ->
          Commands.sequence(
              Commands.runOnce(() -> Logger.recordOutput("Active EE", "ALGAE_GRAB")),
              AlgaeClawCmds.setSpeed(SetpointConstants.AlgaeClaw.ALGAE_GRAB_SPEED));

  public static Supplier<Command> WAIT_UNTIL_ALGAE = () -> AlgaeClawCmds.waitUntilAlgae();

  public static Supplier<Command> ALGAE_HOLD =
      () ->
          Commands.sequence(
              Commands.runOnce(() -> Logger.recordOutput("Active EE", "ALGAE_HOLD")),
              AlgaeClawCmds.setSpeed(SetpointConstants.AlgaeClaw.ALGAE_HOLD_SPEED));

  public static Supplier<Command> ALGAE_GRAB_AND_CORAL_SCORE =
      () ->
          Commands.sequence(
              Commands.runOnce(
                  () -> Logger.recordOutput("Active EE", "ALGAE_GRAB_AND_CORAL_SCORE")),
              AlgaeClawCmds.setSpeed(
                  SetpointConstants.AlgaeClaw.ALGAE_GRAB_SPEED), // run algae motors
              AlgaeClawCmds.waitUntilAlgae(2), // wait for algae sensor
              RollerCmds.setEnableLimitSwitch(false), // score coral
              RollerCmds.setSpeed(SetpointConstants.Roller.L2_PLUS_CORAL_SCORE_SPEED));

  public static Supplier<Command> PROCESSOR_SCORE =
      () ->
          Commands.sequence(
              Commands.runOnce(() -> Logger.recordOutput("Active EE", "PROCESSOR_SCORE")),
              AlgaeClawCmds.setSpeed(SetpointConstants.AlgaeClaw.PROCESSOR_SCORE_SPEED));

  public static Supplier<Command> BARGE_SCORE =
      () ->
          Commands.sequence(
              Commands.runOnce(() -> Logger.recordOutput("Active EE", "BARGE_SCORE")),
              AlgaeClawCmds.setSpeed(SetpointConstants.AlgaeClaw.BARGE_SCORE_SPEED));

  public static Supplier<Command> STOP_ROLLERS =
      () ->
          Commands.sequence(
              Commands.runOnce(() -> Logger.recordOutput("Active EE", "STOP_ROLLERS")),
              RollerCmds.setSpeed(() -> 0),
              AlgaeClawCmds.setSpeed(() -> 0));
}
