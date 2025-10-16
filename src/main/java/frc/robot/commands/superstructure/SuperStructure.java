package frc.robot.commands.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.SetpointConstants;
import frc.robot.commands.AlgaeClawCmds;
import frc.robot.commands.ElevatorCmds;
import frc.robot.commands.IntakeCmds;
import frc.robot.commands.RollerCmds;
import frc.robot.commands.ShoulderCmds;
import frc.robot.subsystems.constants.ElevatorConstants;
import frc.robot.subsystems.constants.IntakeConstants;
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
              SetpointConstants.Intake.PIVOT_DOWN_ANGLE);

  public static Supplier<Command> SOURCE_CORAL_INTAKE =
      () ->
          new SetAllDOFS(
                  "SOURCE_CORAL_INTAKE",
                  "INTAKE_CORAL",
                  () -> true, // ready for coral-ing
                  SetpointConstants.Roller.SOURCE_CORAL_INTAKE_SPEED, // start coral-ing
                  () ->
                      0, // SetpointConstants.AlgaeClaw.ALGAE_HOLD_SPEED, // not actually algae-ing
                  SetpointConstants.Elevator.SOURCE_CORAL_INTAKE_HEIGHT_IN,
                  SetpointConstants.Shoulder.SOURCE_CORAL_INTAKE_ANGLE_DEG,
                  SetpointConstants.Intake.PIVOT_DOWN_ANGLE)
              .andThen(RollerCmds.waitUntilCoral(2.0));

  public static Supplier<Command> SOURCE_CORAL_INTAKE_BLOCKED =
      () ->
          new SetAllDOFS(
                  "SOURCE_CORAL_INTAKE_BLOCKED",
                  "INTAKE_CORAL_BLOCKED",
                  () -> true, // ready for coral-ing
                  SetpointConstants.Roller.SOURCE_CORAL_INTAKE_SPEED, // start coral-ing
                  () ->
                      0, // SetpointConstants.AlgaeClaw.ALGAE_HOLD_SPEED, // not actually algae-ing
                  SetpointConstants.Elevator.SOURCE_CORAL_INTAKE_HEIGHT_IN,
                  SetpointConstants.Shoulder.SOURCE_CORAL_INTAKE_BLOCKED_ANGLE_DEG,
                  SetpointConstants.Intake.PIVOT_DOWN_ANGLE)
              .andThen(RollerCmds.waitUntilCoral(2.0));
  ;

  public static Supplier<Command> L1 =
      () ->
          new SetAllDOFS(
              "L1",
              "ALGAE_GRAB",
              SetpointConstants.AlgaeClaw.ALGAE_GRAB_SPEED,
              SetpointConstants.Elevator.L1_HEIGHT_IN,
              SetpointConstants.Shoulder.L1_ANGLE_DEG,
              SetpointConstants.Intake.PIVOT_DOWN_ANGLE);

  // TODO: Replace intake angle with something other than () -> -10
  public static Supplier<Command> L2 =
      () ->
          new SetAllDOFS(
              "L2",
              "ALGAE_GRAB",
              SetpointConstants.AlgaeClaw.ALGAE_GRAB_SPEED,
              SetpointConstants.Elevator.L2_HEIGHT_IN,
              SetpointConstants.Shoulder.L2_ANGLE_DEG,
              () -> -10);

  public static Supplier<Command> L3 =
      () ->
          new SetAllDOFS(
              "L3",
              "ALGAE_GRAB",
              SetpointConstants.AlgaeClaw.ALGAE_GRAB_SPEED,
              SetpointConstants.Elevator.L3_HEIGHT_IN,
              SetpointConstants.Shoulder.L3_ANGLE_DEG,
              () -> -10);

  public static Supplier<Command> L4_PREP =
      () ->
          new SetAllDOFS(
              "L4_PREP",
              "STOP_ROLLERS",
              () -> 0, // stop algae claw
              SetpointConstants.Elevator.L4_PREP_HEIGHT_IN,
              SetpointConstants.Shoulder.L4_PREP_ANGLE_DEG,
              () -> -10);

  public static Supplier<Command> L4 =
      () ->
          new SetAllDOFS(
              "L4",
              "STOP_ROLLERS",
              () -> 0, // stop algae claw
              SetpointConstants.Elevator.L4_HEIGHT_IN,
              SetpointConstants.Shoulder.L4_ANGLE_DEG,
              () -> -10);

  public static Supplier<Command> BARGE_PREP_FORWARDS =
      () ->
          Commands.sequence(
              new InstantCommand(() -> Logger.recordOutput("Active SS", "BARGE_FORWARDS")),
              AlgaeClawCmds.setSpeed(SetpointConstants.AlgaeClaw.ALGAE_HOLD_SPEED),
              ElevatorCmds.setHeightAndWait(SetpointConstants.Elevator.BARGE_HEIGHT_IN),
              ShoulderCmds.setAngleAndWait(SetpointConstants.Shoulder.BARGE_ANGLE_DEGREES));

  public static Supplier<Command> BARGE_PREP_FORWARDS_AUTO =
      () ->
          Commands.sequence(
              new InstantCommand(() -> Logger.recordOutput("Active SS", "BARGE_FORWARDS_AUTO")),
              AlgaeClawCmds.setSpeed(SetpointConstants.AlgaeClaw.ALGAE_HOLD_SPEED),
              ElevatorCmds.setHeightAndWait(SetpointConstants.Elevator.BARGE_HEIGHT_IN),
              ShoulderCmds.setAngleAndWait(90));

  public static Supplier<Command> BARGE_PREP_BACKWARDS =
      () ->
          Commands.sequence(
              new InstantCommand(() -> Logger.recordOutput("Active SS", "BARGE_BACKWARDS")),
              AlgaeClawCmds.setSpeed(SetpointConstants.AlgaeClaw.ALGAE_HOLD_SPEED),
              ElevatorCmds.setHeightAndWait(SetpointConstants.Elevator.BARGE_HEIGHT_IN),
              ShoulderCmds.setAngleAndWait(
                  SetpointConstants.Shoulder.BARGE_ANGLE_DEGREES_BACKWARDS));

  // TODO: cleanup these algae commands using SetAllDOFS or SetDOFSOneAtATime
  // TODO: use the LoggedTunableNumbers created in SetpointConstants instead of hard-coded values

  // Necessary for picking up algae off the ground- otherwise the ss hits the bumper
  public static Supplier<Command> STARTING_CONF_WITH_ALGAE =
      () ->
          Commands.sequence(
              new InstantCommand(
                  () -> Logger.recordOutput("Active SS", "STARTING_CONF_WITH_ALGAE")),
              new SetAllDOFS(
                  "STARTING_CONF_WITH_ALGAE",
                  "ALGAE_HOLD",
                  SetpointConstants.AlgaeClaw.ALGAE_HOLD_SPEED,
                  SetpointConstants.Elevator.STARTING_HEIGHT,
                  SetpointConstants.Shoulder.ALGAE_STARTING_ANGLE,
                  () -> -10));

  public static Supplier<Command> ALGAE_GRAB_L2 =
      () ->
          Commands.sequence(
              new InstantCommand(() -> Logger.recordOutput("Active SS", "ALGAE_GRAB_L2")),
              ElevatorCmds.setHeightAndWait(
                  SetpointConstants.Elevator.ALGAE_L2_IN), // runs the elevator first
              Commands.parallel(
                  ShoulderCmds.setAngleAndWait(SetpointConstants.Shoulder.ALGAE_L2_DEG),
                  EndEffector.ALGAE_GRAB.get()),
              AlgaeClawCmds.waitUntilAlgae(),
              EndEffector.ALGAE_HOLD.get());

  public static Supplier<Command> ALGAE_SS_L2 =
      () ->
          Commands.sequence(
              new InstantCommand(() -> Logger.recordOutput("Active SS", "ALGAE_SS_L2")),
              ElevatorCmds.setHeightAndWait(
                  SetpointConstants.Elevator.ALGAE_L2_IN), // runs the elevator first
              Commands.parallel(EndEffector.ALGAE_GRAB.get(), ShoulderCmds.setAngleAndWait(110)));

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
              Commands.parallel(
                  ShoulderCmds.setAngleAndWait(SetpointConstants.Shoulder.ALGAE_L3_DEG),
                  EndEffector.ALGAE_GRAB.get()),
              AlgaeClawCmds.waitUntilAlgae(),
              EndEffector.ALGAE_HOLD.get());

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
              AlgaeClawCmds.setSpeed(SetpointConstants.AlgaeClaw.ALGAE_GRAB_SPEED),
              ShoulderCmds.setAngleAndWait(SetpointConstants.Shoulder.ALGAE_GROUND_DEG),
              EndEffector.ALGAE_GRAB.get(),
              AlgaeClawCmds.waitUntilAlgae(),
              SuperStructure.STARTING_CONF_WITH_ALGAE.get(),
              EndEffector.ALGAE_HOLD.get());

  public static Supplier<Command> ALGAE_SAFE_RETRACT =
      () ->
          new SetDOFSOneAtATimeFactory("ALGAE_SAFE_RETRACT", "ALGAE_HOLD")
              .addAlgaeSpeedCommand(SetpointConstants.AlgaeClaw.ALGAE_HOLD_SPEED)
              .create();

  public static Supplier<Command> PROCESSOR_PREP =
      () ->
          new SetDOFSOneAtATimeFactory("PROCESSOR_PREP", "ALGAE_HOLD")
              .addAlgaeSpeedCommand(SetpointConstants.AlgaeClaw.ALGAE_HOLD_SPEED)
              .addShoulderCommand(SetpointConstants.Shoulder.PROCESSOR_SCORE_ANGLE_DEG)
              .addElevatorCommand(SetpointConstants.Elevator.PROCESSOR_HEIGHT_IN)
              .create();

  public static Supplier<Command> PROCESSOR_PREP_BACKWARDS =
      () ->
          new SetDOFSOneAtATimeFactory("PROCESSOR_PREP_BACKWARDS", "ALGAE_HOLD")
              .addAlgaeSpeedCommand(SetpointConstants.AlgaeClaw.ALGAE_HOLD_SPEED)
              .addShoulderCommand(() -> -104)
              .addElevatorCommand(SetpointConstants.Elevator.PROCESSOR_HEIGHT_IN)
              .create();

  public static Supplier<Command> INTAKE_ROLLER_RUN =
      () -> Commands.sequence(IntakeCmds.setVolts(SetpointConstants.Intake.ROLLER_SPEED));

  public static Supplier<Command> INTAKE_ROLLER_STOP =
      () -> Commands.sequence(IntakeCmds.setVolts(0));

  public static Supplier<Command> INTAKE_PIVOT_RAISE =
      () ->
          Commands.sequence(
              ElevatorCmds.setSoftMinHeight(ElevatorConstants.kMinIntakeUpHeight),
              new InstantCommand(
                  () -> {
                    if (RobotContainer.elevator.getCurrentHeight() >= 5) {
                      ElevatorCmds.setHeight(ElevatorConstants.kMinIntakeUpHeight);
                    }
                  }),
              // TODO: change to a value based on real measurements, and set it in IntakeConstants
              // rather than here
              IntakeCmds.setAngle(SetpointConstants.Intake.PIVOT_UP_ANGLE));
  public static Supplier<Command> INTAKE_PIVOT_LOWER =
      () ->
          Commands.sequence(
              IntakeCmds.setAngle(IntakeConstants.kIPInitialAngleDeg),
              IntakeCmds.setAngle(SetpointConstants.Intake.PIVOT_DOWN_ANGLE));
  public static Supplier<Command> CORAL_GRAB_GROUND =
      () -> Commands.sequence(IntakeCmds.setVolts(IntakeConstants.kRollerGrabSpeed));
}
