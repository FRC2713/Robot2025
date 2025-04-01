package frc.robot.commands.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.SetpointConstants;
import frc.robot.commands.AlgaeClawCmds;
import frc.robot.commands.ElevatorCmds;
import frc.robot.commands.PivotCmds;
import frc.robot.commands.RollerCmds;
import frc.robot.commands.ShoulderCmds;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

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
                  SetpointConstants.AlgaeClaw.ALGAE_HOLD_SPEED, // not actually algae-ing
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

  // TODO: We don't necessarily need to start the algae claw here
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

  public static Supplier<Command> BARGE_PREP_FORWARDS =
      () ->
          Commands.sequence(
              new InstantCommand(() -> Logger.recordOutput("Active SS", "BARGE_BACKWARDS")),
              AlgaeClawCmds.setSpeed(SetpointConstants.AlgaeClaw.ALGAE_HOLD_SPEED),
              ElevatorCmds.setHeightAndWait(SetpointConstants.Elevator.BARGE_HEIGHT_IN),
              PivotCmds.setAngle(90),
              ShoulderCmds.setAngleAndWait(SetpointConstants.Shoulder.BARGE_ANGLE_DEGREES),
              PivotCmds.setAngleAndWait(SetpointConstants.Pivot.BARGE_ANGLE_DEG));

  public static Supplier<Command> BARGE_PREP_BACKWARDS =
      () ->
          Commands.sequence(
              new InstantCommand(() -> Logger.recordOutput("Active SS", "BARGE_BACKWARDS")),
              AlgaeClawCmds.setSpeed(SetpointConstants.AlgaeClaw.ALGAE_HOLD_SPEED),
              ElevatorCmds.setHeightAndWait(SetpointConstants.Elevator.BARGE_HEIGHT_IN),
              PivotCmds.setAngle(90),
              ShoulderCmds.setAngleAndWait(
                  SetpointConstants.Shoulder.BARGE_ANGLE_DEGREES_BACKWARDS),
              PivotCmds.setAngleAndWait(SetpointConstants.Pivot.BARGE_ANGLE_DEG_BACKWARDS));

  // TODO: cleanup these algae commands using SetAllDOFS or SetDOFSOneAtATime
  // TODO: use the LoggedTunableNumbers created in SetpointConstants instead of hard-coded values

  // Necessary for picking up algae off the ground- otherwise the ss hits the bumper
  public static Supplier<Command> STARTING_CONF_WITH_ALGAE =
      () ->
          Commands.sequence(
              new InstantCommand(
                  () -> Logger.recordOutput("Active SS", "STARTING_CONF_WITH_ALGAE")),
              PivotCmds.setAngleAndWait(
                  SetpointConstants.Pivot.ALGAE_STARTING_ANGLE), // runs the pivot first
              new SetAllDOFS(
                  "STARTING_CONF_WITH_ALGAE",
                  "ALGAE_HOLD",
                  SetpointConstants.AlgaeClaw.ALGAE_HOLD_SPEED,
                  SetpointConstants.Elevator.STARTING_HEIGHT,
                  SetpointConstants.Shoulder.ALGAE_STARTING_ANGLE,
                  SetpointConstants.Pivot.ALGAE_STARTING_ANGLE));

  public static Supplier<Command> ALGAE_GRAB_L2 =
      () ->
          Commands.sequence(
              new InstantCommand(() -> Logger.recordOutput("Active SS", "ALGAE_GRAB_L2")),
              ElevatorCmds.setHeightAndWait(
                  SetpointConstants.Elevator.ALGAE_L2_IN), // runs the elevator first
              new SetAllDOFS(
                  "ALGAE_GRAB_L2",
                  "ALGAE_GRAB",
                  SetpointConstants.AlgaeClaw.ALGAE_GRAB_SPEED,
                  SetpointConstants.Elevator.ALGAE_L2_IN,
                  SetpointConstants.Shoulder.ALGAE_L2_DEG,
                  SetpointConstants.Pivot.ALGAE_L2_DEG));

  public static Supplier<Command> ALGAE_COLLECT_L2 =
      () ->
          Commands.sequence(
              new InstantCommand(() -> Logger.recordOutput("Active SS", "ALGAE_COLLECT_L2")),
              EndEffector.ALGAE_GRAB.get(),
              EndEffector.WAIT_UNTIL_ALGAE.get(),
              EndEffector.ALGAE_HOLD.get(),
              SuperStructure.STARTING_CONF.get());

  public static Supplier<Command> ALGAE_GRAB_L3 =
      () ->
          Commands.sequence(
              new InstantCommand(() -> Logger.recordOutput("Active SS", "ALGAE_GRAB_L3")),
              ElevatorCmds.setHeightAndWait(
                  SetpointConstants.Elevator.ALGAE_L3_IN), // runs the elevator first
              new SetAllDOFS(
                  "ALGAE_GRAB_L3",
                  "ALGAE_GRAB",
                  SetpointConstants.AlgaeClaw.ALGAE_GRAB_SPEED,
                  SetpointConstants.Elevator.ALGAE_L3_IN,
                  SetpointConstants.Shoulder.ALGAE_L3_DEG,
                  SetpointConstants.Pivot.ALGAE_L3_DEG));

  public static Supplier<Command> ALGAE_COLLECT_L3 =
      () ->
          Commands.sequence(
              new InstantCommand(() -> Logger.recordOutput("Active SS", "ALGAE_COLLECT_L2")),
              EndEffector.ALGAE_GRAB.get(),
              EndEffector.WAIT_UNTIL_ALGAE.get(),
              EndEffector.ALGAE_HOLD.get(),
              SuperStructure.STARTING_CONF.get());

  public static Supplier<Command> ALGAE_GRAB_GROUND =
      () ->
          Commands.sequence(
              new InstantCommand(() -> Logger.recordOutput("Active SS", "ALGAE_GRAB_GROUND")),
              ElevatorCmds.setHeightAndWait(
                  SetpointConstants.Elevator.ALGAE_GROUND_IN), // runs the elevator first
              new SetAllDOFS(
                  "ALGAE_GRAB_GROUND",
                  "ALGAE_GRAB",
                  SetpointConstants.AlgaeClaw.ALGAE_GRAB_SPEED,
                  SetpointConstants.Elevator.ALGAE_GROUND_IN,
                  SetpointConstants.Shoulder.ALGAE_GROUND_DEG,
                  SetpointConstants.Pivot.ALGAE_GROUND_DEG));

  public static Supplier<Command> ALGAE_SAFE_RETRACT =
      () ->
          new SetDOFSOneAtATimeFactory("ALGAE_SAFE_RETRACT", "ALGAE_HOLD")
              .addAlgaeSpeedCommand(SetpointConstants.AlgaeClaw.ALGAE_HOLD_SPEED)
              .addPivotCommand(SetpointConstants.Pivot.SAFE_ANGLE_DEGS)
              .create();

  public static Supplier<Command> PROCESSOR_PREP =
      () ->
          new SetDOFSOneAtATimeFactory("PROCESSOR_PREP", "ALGAE_HOLD")
              .addAlgaeSpeedCommand(SetpointConstants.AlgaeClaw.ALGAE_HOLD_SPEED)
              .addShoulderCommand(SetpointConstants.Shoulder.PROCESSOR_SCORE_ANGLE_DEG)
              .addPivotCommand(SetpointConstants.Pivot.PROCESSOR_SCORE_ANGLE_DEG)
              .addElevatorCommand(SetpointConstants.Elevator.PROCESSOR_HEIGHT_IN)
              .create();
}
