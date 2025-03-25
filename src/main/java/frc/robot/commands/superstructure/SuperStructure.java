package frc.robot.commands.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.SetpointConstants;
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

  // TODO: if this intersects with the reef, might need to do pivot last
  public static Supplier<Command> L4_PREP =
      () ->
          new SetAllDOFS(
              "L4_PREP",
              "STOP_ROLLERS",
              () -> 0, // stop algae claw
              SetpointConstants.Elevator.L4_PREP_HEIGHT_IN,
              SetpointConstants.Shoulder.L4_PREP_ANGLE_DEG,
              SetpointConstants.Pivot.L4_ANGLE_DEG);

  // TODO: if this intersects with the reef, might need to do pivot last
  public static Supplier<Command> L4 =
      () ->
          new SetAllDOFS(
              "L4",
              "STOP_ROLLERS",
              () -> 0, // stop algae claw
              SetpointConstants.Elevator.L4_HEIGHT_IN,
              SetpointConstants.Shoulder.L4_ANGLE_DEG,
              SetpointConstants.Pivot.L4_ANGLE_DEG);

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

  // TODO: cleanup these algae commands using SetAllDOFS or SetDOFSOneAtATime
  // TODO: use the LoggedTunableNumbers created in SetpointConstants instead of hard-coded values

  // Necessary for picking up algae off the ground- otherwise the ss hits the bumper
  public static Supplier<Command> STARTING_CONF_WITH_ALGAE =
      () ->
          new SetDOFSOneAtATimeFactory("STARTING_CONF_WITH_ALGAE", "STOP_ROLLERS")
              .addElevatorCommand(SetpointConstants.Elevator.STARTING_HEIGHT)
              .addPivotCommand(SetpointConstants.Pivot.STARTING_ANGLE)
              .addAlgaeSpeedCommand(() -> 0)
              .addCoralSpeedCommand(() -> 0)
              .addShoulderCommand(SetpointConstants.Shoulder.STARTING_ANGLE)
              .create();

  public static Supplier<Command> ALGAE_GRAB_L2 =
      () ->
          new SetDOFSOneAtATimeFactory(null, null)
              .addAlgaeSpeedCommand(SetpointConstants.AlgaeClaw.ALGAE_GRAB_SPEED)
              .addElevatorCommand(SetpointConstants.Elevator.ALGAE_L2_IN)
              .addShoulderCommand(SetpointConstants.Shoulder.ALGAE_L2_DEG)
              .addPivotCommand(SetpointConstants.Pivot.ALGAE_L2_DEG)
              .create();

  public static Supplier<Command> ALGAE_GRAB_L3 =
      () ->
          new SetDOFSOneAtATimeFactory(null, null)
              .addAlgaeSpeedCommand(SetpointConstants.AlgaeClaw.ALGAE_GRAB_SPEED)
              .addElevatorCommand(SetpointConstants.Elevator.ALGAE_L3_IN)
              .addShoulderCommand(SetpointConstants.Shoulder.ALGAE_L3_DEG)
              .addPivotCommand(SetpointConstants.Pivot.ALGAE_L3_DEG)
              .create();

  public static Supplier<Command> ALGAE_GRAB_GROUND =
      () ->
      new SetDOFSOneAtATimeFactory(null, null)
      .addAlgaeSpeedCommand(SetpointConstants.AlgaeClaw.ALGAE_GRAB_SPEED)
      .addElevatorCommand(SetpointConstants.Elevator.ALGAE_GROUND_IN)
      .addShoulderCommand(SetpointConstants.Shoulder.ALGAE_GROUND_DEG)
      .addPivotCommand(SetpointConstants.Pivot.ALGAE_GROUND_DEG).create();

  public static Supplier<Command> ALGAE_SAFE_RETRACT =
      () -> Commands.sequence(PivotCmds.setAngle(SetpointConstants.Pivot.SAFE_ANGLE_DEGS));

  public static Supplier<Command> PROCESSOR_PREP =
      () ->
      new SetDOFSOneAtATimeFactory(null, null)
      .addAlgaeSpeedCommand(SetpointConstants.AlgaeClaw.ALGAE_HOLD_SPEED)
      .addShoulderCommand(SetpointConstants.Shoulder.ALGAE_L2_DEG)
      .addPivotCommand(SetpointConstants.Pivot.PROCESSOR_SCORE_ANGLE_DEG)
      .addElevatorCommand(SetpointConstants.Elevator.ALGAE_L2_IN)
      .create();
}
