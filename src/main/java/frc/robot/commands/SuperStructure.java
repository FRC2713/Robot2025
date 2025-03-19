package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.SSConstants;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public enum SuperStructure {
  STARTING_CONF(
      () ->
          new SetAllDOFS(
              () -> true, // ready for coral-ing
              () -> 0, // not actually coral-ing
              () -> 0, // not actually algae-ing
              SSConstants.Elevator.STARTING_HEIGHT,
              SSConstants.Shoulder.STARTING_ANGLE,
              SSConstants.Pivot.STARTING_ANGLE)),

  SOURCE_CORAL_INTAKE(
      () ->
          new SetAllDOFS(
                  () -> true, // ready for coral-ing
                  SSConstants.Roller.SOURCE_CORAL_INTAKE_SPEED, // start coral-ing
                  () -> 0, // not actually algae-ing
                  SSConstants.Elevator.SOURCE_CORAL_INTAKE_HEIGHT_IN,
                  SSConstants.Shoulder.SOURCE_CORAL_INTAKE_ANGLE_DEG,
                  SSConstants.Pivot.SOURCE_CORAL_INTAKE_ANGLE_DEG)
              .andThen(RollerCmds.waitUntilCoral(2.0))),

  L1(
      () ->
          new SetAllDOFS(
              SSConstants.Elevator.L1_HEIGHT_IN,
              SSConstants.Shoulder.L1_ANGLE_DEG,
              SSConstants.Pivot.L1_ANGLE_DEG)),

  L2(
      () ->
          new SetAllDOFS(
              SSConstants.Elevator.L2_HEIGHT_IN,
              SSConstants.Shoulder.L2_ANGLE_DEG,
              SSConstants.Pivot.L2_ANGLE_DEG)),
  L3(
      () ->
          new SetAllDOFS(
              SSConstants.Elevator.L3_HEIGHT_IN,
              SSConstants.Shoulder.L3_ANGLE_DEG,
              SSConstants.Pivot.L3_ANGLE_DEG)),

  L4_PREP(
      () ->
          new SetAllDOFS(
              () -> 0, // stop algae claw
              SSConstants.Elevator.L4_PREP_HEIGHT_IN,
              SSConstants.Shoulder.L4_ANGLE_DEG,
              SSConstants.Pivot.L4_ANGLE_DEG)),

  L4(
      () ->
          new SetAllDOFS(
              () -> 0, // stop algae claw
              SSConstants.Elevator.L4_HEIGHT_IN,
              SSConstants.Shoulder.L4_ANGLE_DEG,
              SSConstants.Pivot.L4_ANGLE_DEG)),

  PROCESSOR(
      () ->
          new SetDOFSOneAtATime(
                  SSConstants.AlgaeClaw.ALGAE_HOLD_SPEED,
                  SSConstants.Elevator.PROCESSOR_PREP_HEIGHT_IN,
                  SSConstants.Shoulder.PROCESSOR_SCORE_ANGLE_DEG,
                  SSConstants.Pivot.PROCESSOR_SCORE_ANGLE_DEG)
              .andThen(
                  ElevatorCmds.setHeightAndWait(SSConstants.Elevator.PROCESSOR_SCORE_HEIGHT_IN))),

  ALGAE_SAFE_RETRACT(() -> PivotCmds.setAngle(SSConstants.Pivot.SAFE_ANGLE_DEGS)),

  CLIMBING_CONF(
      () ->
          Commands.sequence(
              // move the arm out of the way, elevator first
              Commands.parallel(
                  ElevatorCmds.setHeightAndWait(SSConstants.Elevator.CLIMB_PREP_HEIGHT),
                  ShoulderCmds.setAngle(SSConstants.Shoulder.PREP_CLIMB_ANGLE_DEGS),
                  PivotCmds.setAngle(SSConstants.Pivot.PREP_CLIMB_ANGLE_DEGS)),
              // deploy the climber
              ClimberCmds.deploy(),
              Commands.waitSeconds(0.5),
              // tuck the arm back in
              Commands.sequence(
                  ShoulderCmds.setAngle(SSConstants.Shoulder.CLIMB_ANGLE_DEGS),
                  PivotCmds.setAngle(SSConstants.Pivot.CLIMB_ANGLE_DEGS),
                  ElevatorCmds.setHeight(0)))),
  BARGE(
      () ->
          new SetAllDOFS(
              SSConstants.AlgaeClaw.ALGAE_HOLD_SPEED,
              SSConstants.Elevator.BARGE_HEIGHT_IN,
              SSConstants.Shoulder.BARGE_ANGLE_DEGREES,
              SSConstants.Pivot.BARGE_ANGLE_DEG));

  private Supplier<Command> cmd;

  private SuperStructure(Supplier<Command> cmd) {
    this.cmd = cmd;
  }

  public Command getCommand() {
    return Commands.sequence(
        new InstantCommand(() -> Logger.recordOutput("Active SS", this.toString())),
        Commands.print("Starting SS: " + this.toString()),
        cmd.get());
  }

  public Command delayCommand(double seconds) {
    return Commands.sequence(Commands.waitSeconds(seconds), getCommand());
  }
}
