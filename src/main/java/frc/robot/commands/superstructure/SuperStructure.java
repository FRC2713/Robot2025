package frc.robot.commands.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SetpointConstants;
import frc.robot.commands.ElevatorCmds;
import frc.robot.commands.PivotCmds;
import frc.robot.commands.RollerCmds;
import java.util.function.Supplier;

public class SuperStructure {

  public static Supplier<Command> STARTING_CONF =
      () ->
          new SetAllDOFS(
              "STARTING_CONFG",
              "STOP_ROLLERS",
              () -> true, // ready for coral-ing
              () -> 0, // not actually coral-ing
              () -> 0, // not actually algae-ing
              SetpointConstants.Elevator.STARTING_HEIGHT,
              SetpointConstants.Shoulder.STARTING_ANGLE,
              SetpointConstants.Pivot.STARTING_ANGLE);

  public static Supplier<Command> SOURCE_CORAL_INTAKE =
      () ->
          new SetAllDOFS(
                  "SOURCE_CORAL_INTAKE",
                  "INTAKE_CORAL",
                  () -> true, // ready for coral-ing
                  SetpointConstants.Roller.SOURCE_CORAL_INTAKE_SPEED, // start coral-ing
                  () -> 0, // not actually algae-ing
                  SetpointConstants.Elevator.SOURCE_CORAL_INTAKE_HEIGHT_IN,
                  SetpointConstants.Shoulder.SOURCE_CORAL_INTAKE_ANGLE_DEG,
                  SetpointConstants.Pivot.SOURCE_CORAL_INTAKE_ANGLE_DEG)
              .andThen(RollerCmds.waitUntilCoral(2.0));

  public static Supplier<Command> L1 =
      () ->
          new SetAllDOFS(
              "L1",
              "ALGAE_GRAB",
              SetpointConstants.AlgaeClaw.ALGAE_GRAB_SPEED,
              SetpointConstants.Elevator.L1_HEIGHT_IN,
              SetpointConstants.Shoulder.L1_ANGLE_DEG,
              SetpointConstants.Pivot.L1_ANGLE_DEG);

  public static Supplier<Command> L2 =
      () ->
          new SetAllDOFS(
              "L2",
              "ALGAE_GRAB",
              SetpointConstants.AlgaeClaw.ALGAE_GRAB_SPEED,
              SetpointConstants.Elevator.L2_HEIGHT_IN,
              SetpointConstants.Shoulder.L2_ANGLE_DEG,
              SetpointConstants.Pivot.L2_ANGLE_DEG);

  public static Supplier<Command> L3 =
      () ->
          new SetAllDOFS(
              "L3",
              "ALGAE_GRAB",
              SetpointConstants.AlgaeClaw.ALGAE_GRAB_SPEED,
              SetpointConstants.Elevator.L3_HEIGHT_IN,
              SetpointConstants.Shoulder.L3_ANGLE_DEG,
              SetpointConstants.Pivot.L3_ANGLE_DEG);

  public static Supplier<Command> L4_PREP =
      () ->
          new SetAllDOFS(
              "L4_PREP",
              "ALGAE_GRAB",
              () -> 0, // stop algae claw
              SetpointConstants.Elevator.L4_PREP_HEIGHT_IN,
              SetpointConstants.Shoulder.L4_ANGLE_DEG,
              SetpointConstants.Pivot.L4_ANGLE_DEG);

  public static Supplier<Command> L4 =
      () ->
          new SetAllDOFS(
              "L4",
              "ALGAE_GRAB",
              () -> 0, // stop algae claw
              SetpointConstants.Elevator.L4_HEIGHT_IN,
              SetpointConstants.Shoulder.L4_ANGLE_DEG,
              SetpointConstants.Pivot.L4_ANGLE_DEG);

  public static Supplier<Command> PROCESSOR =
      () ->
          new SetDOFSOneAtATime(
                  "PROCESSOR",
                  "ALGAE_HOLD",
                  SetpointConstants.AlgaeClaw.ALGAE_HOLD_SPEED,
                  SetpointConstants.Elevator.PROCESSOR_PREP_HEIGHT_IN,
                  SetpointConstants.Shoulder.PROCESSOR_SCORE_ANGLE_DEG,
                  SetpointConstants.Pivot.PROCESSOR_SCORE_ANGLE_DEG)
              .andThen(
                  ElevatorCmds.setHeightAndWait(
                      SetpointConstants.Elevator.PROCESSOR_SCORE_HEIGHT_IN));

  public static Supplier<Command> ALGAE_SAFE_RETRACT =
      () -> PivotCmds.setAngle(SetpointConstants.Pivot.SAFE_ANGLE_DEGS);

  public static Supplier<Command> CLIMBING_CONF = () -> new SetClimbingConfig("CLIMBING_CONF");

  public static Supplier<Command> BARGE =
      () ->
          new SetAllDOFS(
              "BARGE",
              "ALGAE_HOLD",
              SetpointConstants.AlgaeClaw.ALGAE_HOLD_SPEED,
              SetpointConstants.Elevator.BARGE_HEIGHT_IN,
              SetpointConstants.Shoulder.BARGE_ANGLE_DEGREES,
              SetpointConstants.Pivot.BARGE_ANGLE_DEG);
}
