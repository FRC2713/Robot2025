package frc.robot.commands.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.SetpointConstants;
import frc.robot.commands.AlgaeClawCmds;
import frc.robot.commands.ArmCmds;
import frc.robot.commands.ElevatorCmds;
import frc.robot.commands.IntakeCmds;
import frc.robot.commands.RollerCmds;
import frc.robot.commands.ShoulderCmds;
import frc.robot.subsystems.constants.IntakeConstants;
import frc.robot.util.ScoreLevel;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class SuperStructure {

  public static Supplier<Command> STARTING_CONF =
      () ->
          new SequentialCommandGroup(
              IntakeCmds.setVolts(0),
              ArmCmds.handSetVoltage(0),
              ElevatorCmds.setHeightAndWait(
                  SetpointConstants.Elevator.ELEVATOR_CORAL_HANDOFF_HEIGHT),
              IntakeCmds.setAngleAndWait(SetpointConstants.Intake.INTAKE_HANDOFF_ANGLE),
              ArmCmds.armSetAngle(-90));

  public static Supplier<Command> ALGAE_CONF =
      () ->
          new SequentialCommandGroup(
              IntakeCmds.setVolts(0),
              ArmCmds.handSetVoltage(0),
              ElevatorCmds.setHeightAndWait(
                  SetpointConstants.Elevator.ELEVATOR_ALGAE_HANDOFF_HEIGHT),
              IntakeCmds.setAngleAndWait(SetpointConstants.Intake.INTAKE_HANDOFF_ANGLE),
              ArmCmds.armSetAngle(-90));

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
                  SetpointConstants.Shoulder.SOURCE_CORAL_INTAKE_ANGLE_DEG)
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
                  SetpointConstants.Shoulder.SOURCE_CORAL_INTAKE_BLOCKED_ANGLE_DEG)
              .andThen(RollerCmds.waitUntilCoral(2.0));
  ;

  public static Supplier<Command> L1 =
      () ->
          Commands.sequence(
              new InstantCommand(() -> RobotContainer.scoreLevel = ScoreLevel.ONE),
              IntakeCmds.setAngleAndWait(SetpointConstants.Intake.INTAKE_L1_ANGLE));

  // TODO: We don't necessarily need to start the algae claw here
  public static Supplier<Command> L2 =
      () ->
          new SetAllDOFS(
              "L2",
              "ALGAE_GRAB",
              SetpointConstants.AlgaeClaw.ALGAE_GRAB_SPEED,
              SetpointConstants.Elevator.L2_HEIGHT_IN,
              SetpointConstants.Shoulder.L2_ANGLE_DEG);

  public static Supplier<Command> L3 =
      () ->
          Commands.sequence(
              new InstantCommand(() -> RobotContainer.scoreLevel = ScoreLevel.THREE),
              IntakeCmds.setVolts(-10),
              ArmCmds.handSetVoltage(-10),
              ArmCmds.handWaitUntilCoral(2),
              ArmCmds.handSetVoltage(-2),
              IntakeCmds.setVolts(0.),
              ArmCmds.armSetAngleAndWait(SetpointConstants.Arm.L3_ANGLE_DEG),
              ElevatorCmds.setHeight(SetpointConstants.Elevator.L3_HEIGHT_IN));

  public static Supplier<Command> AGLAE_PROCESSOR =
      () ->
          Commands.sequence(
              new InstantCommand(() -> RobotContainer.scoreLevel = ScoreLevel.ONE),
              IntakeCmds.setAngleAndWait(SetpointConstants.Intake.INTAKE_PROCESSOR_ANGLE));

  public static Supplier<Command> ALGAE_SCORE =
      () ->
          Commands.sequence(
              new InstantCommand(() -> RobotContainer.scoreLevel = ScoreLevel.FOUR),
              IntakeCmds.setVolts(-10),
              ArmCmds.handSetVoltage(-10),
              ArmCmds.handWaitUntilAlgae(2),
              ArmCmds.handSetVoltage(-2),
              IntakeCmds.setVolts(0.),
              ArmCmds.armSetAngleAndWait(SetpointConstants.Arm.BARGE_ANGLE_SCORE),
              ElevatorCmds.setHeight(SetpointConstants.Elevator.BARGE_HEIGHT_SCORE));

  // TODO: if this intersects with the reef, might need to do pivot last
  public static Supplier<Command> L4_PREP =
      () ->
          new SetAllDOFS(
              "L4_PREP",
              "STOP_ROLLERS",
              () -> 0, // stop algae claw
              SetpointConstants.Elevator.L4_PREP_HEIGHT_IN,
              SetpointConstants.Shoulder.L4_PREP_ANGLE_DEG);

  // TODO: if this intersects with the reef, might need to do pivot last
  public static Supplier<Command> L4 =
      () ->
          Commands.sequence(
              new InstantCommand(() -> RobotContainer.scoreLevel = ScoreLevel.FOUR),
              IntakeCmds.setVolts(-10),
              ArmCmds.handSetVoltage(-10),
              ArmCmds.handWaitUntilCoral(2),
              ArmCmds.handSetVoltage(-2),
              IntakeCmds.setVolts(0.),
              ArmCmds.armSetAngleAndWait(SetpointConstants.Arm.L4_ANGLE_DEG),
              ElevatorCmds.setHeight(SetpointConstants.Elevator.L4_HEIGHT_IN));

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
                  SetpointConstants.Shoulder.ALGAE_STARTING_ANGLE));

  public static Supplier<Command> ALGAE_GRAB_L2 =
      () ->
          Commands.sequence(
              new InstantCommand(() -> RobotContainer.scoreLevel = ScoreLevel.THREE),
              ArmCmds.handSetVoltage(-3),
              ArmCmds.armSetAngleAndWait(SetpointConstants.Arm.L2_ALGAE_INTAKE),
              ElevatorCmds.setHeight(SetpointConstants.Elevator.L3_HEIGHT_IN));

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
              new InstantCommand(() -> RobotContainer.scoreLevel = ScoreLevel.THREE),
              ArmCmds.handSetVoltage(-3),
              ArmCmds.armSetAngleAndWait(SetpointConstants.Arm.L3_ALGAE_INTAKE),
              ElevatorCmds.setHeight(SetpointConstants.Elevator.L3_HEIGHT_IN),
              ArmCmds.handWaitUntilAlgae(30),
              ArmCmds.handSetVoltage(-5));

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

  public static Supplier<Command> PRE_DISABLE_CHECK =
      () ->
          new SequentialCommandGroup(
              ArmCmds.armSetAngleAndWait(-90),
              IntakeCmds.setAngleAndWait(185),
              ElevatorCmds.setHeightAndWait(0),
              IntakeCmds.setAngleAndWait(70));

  public static Supplier<Command> CORAL_GRAB_GROUND =
      () ->
          Commands.sequence(
              //
              ElevatorCmds.setHeightAndWait(
                  SetpointConstants.Elevator.ELEVATOR_CORAL_HANDOFF_HEIGHT),
              Commands.parallel(
                  ArmCmds.armSetAngleAndWait(-90),
                  Commands.sequence(
                      IntakeCmds.setAngle(SetpointConstants.Intake.INTAKE_GRAB_ANGLE),
                      IntakeCmds.setVoltsUntilCoral(SetpointConstants.Intake.INTAKE_GRAB_SPEED),
                      IntakeCmds.setVolts(2),
                      new WaitCommand(0.1),
                      IntakeCmds.setAngleAndWait(SetpointConstants.Intake.INTAKE_HANDOFF_ANGLE))));

  public static Supplier<Command> ALGAE_INTAKE =
      () ->
          Commands.sequence(
              //
              ElevatorCmds.setHeightAndWait(
                  SetpointConstants.Elevator.ELEVATOR_ALGAE_HANDOFF_HEIGHT),
              Commands.parallel(
                  ArmCmds.armSetAngleAndWait(-90),
                  Commands.sequence(
                      IntakeCmds.setAngle(SetpointConstants.Intake.ALGAE_GRAB_ANGLE),
                      IntakeCmds.setVoltsUntilCoral(SetpointConstants.Intake.ALGAE_GRAB_SPEED),
                      IntakeCmds.setVolts(2),
                      new WaitCommand(0.1),
                      IntakeCmds.setAngleAndWait(SetpointConstants.Intake.INTAKE_HANDOFF_ANGLE))));

  public static Supplier<Command> UNFOLD =
      () ->
          Commands.sequence(
              IntakeCmds.setAngleAndWait(IntakeConstants.kIPMaxAngle - 5),
              ElevatorCmds.setHeightAndWait(
                  SetpointConstants.Elevator.ELEVATOR_CORAL_HANDOFF_HEIGHT),
              IntakeCmds.setAngle(SetpointConstants.Intake.INTAKE_HANDOFF_ANGLE),
              ArmCmds.armSetAngle(-90));

  public static Supplier<Command> FOLD =
      () ->
          Commands.sequence(
              IntakeCmds.setAngleAndWait(IntakeConstants.kIPMaxAngle - 5),
              ArmCmds.armSetAngleAndWait(-90),
              ElevatorCmds.setHeightAndWait(SetpointConstants.Elevator.STARTING_HEIGHT),
              IntakeCmds.setAngle(SetpointConstants.Intake.INTAKE_HANDOFF_ANGLE));
}
