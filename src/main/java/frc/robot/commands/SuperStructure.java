package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.SSConstants;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public enum SuperStructure {
  // Loc_GamePiece_Action
  STARTING_CONF(
      () ->
          Commands.sequence(
              RollerCmds.setEnableLimitSwitch(true),
              RollerCmds.setSpeed(() -> 0),
              Commands.either(
                  PivotCmds.setAngleAndWait(() -> 35),
                  Commands.none(),
                  () -> RobotContainer.pivot.getCurrentAngle() < -120),
              Commands.either(
                  PivotCmds.setAngle(0),
                  Commands.none(),
                  () -> RobotContainer.shoulder.getCurrentAngle() < -95),
              Commands.parallel(
                  ElevatorCmds.setHeightAndWait(() -> 0), ShoulderCmds.setAngleAndWait(() -> -90)),
              PivotCmds.setAngleAndWait(SSConstants.Pivot.SOURCE_CORAL_INTAKE_ANGLE_DEG))),
  SOURCE_CORAL_INTAKE(
      () ->
          Commands.sequence(
              RollerCmds.setEnableLimitSwitch(true),
              RollerCmds.setSpeed(SSConstants.Roller.SOURCE_CORAL_INTAKE_SPEED),
              Commands.either(
                  PivotCmds.setAngleAndWait(() -> 35),
                  Commands.none(),
                  () -> RobotContainer.pivot.getCurrentAngle() < -120),
              Commands.parallel(
                  ElevatorCmds.setHeightAndWait(SSConstants.Elevator.SOURCE_CORAL_INTAKE_HEIGHT_IN),
                  ShoulderCmds.setAngleAndWait(SSConstants.Shoulder.SOURCE_CORAL_INTAKE_ANGLE_DEG)),
              PivotCmds.setAngleAndWait(SSConstants.Pivot.SOURCE_CORAL_INTAKE_ANGLE_DEG),
              RollerCmds.waitUntilCoral(2))),
  L1_PREP(
      () ->
          Commands.sequence(
              Commands.either(
                  PivotCmds.setAngleAndWait(() -> 35),
                  Commands.none(),
                  () -> RobotContainer.pivot.getCurrentAngle() < -120),
              ElevatorCmds.setHeightAndWait(SSConstants.Elevator.L1_HEIGHT_IN),
              Commands.parallel(
                  PivotCmds.setAngle(SSConstants.Pivot.L1_ANGLE_DEG),
                  ShoulderCmds.setAngleAndWait(SSConstants.Shoulder.L1_ANGLE_DEG)),
              PivotCmds.waitUntilAtTarget())),
  L2_PREP(
      () ->
          Commands.sequence(
              Commands.either(
                  PivotCmds.setAngleAndWait(() -> 35),
                  Commands.none(),
                  () -> RobotContainer.pivot.getCurrentAngle() < -120),
              ElevatorCmds.setHeightAndWait(SSConstants.Elevator.L2_HEIGHT_IN),
              Commands.parallel(
                  PivotCmds.setAngle(SSConstants.Pivot.L2_ANGLE_DEG),
                  ShoulderCmds.setAngleAndWait(SSConstants.Shoulder.L2_ANGLE_DEG)),
              PivotCmds.waitUntilAtTarget())),
  L3_PREP(
      () ->
          Commands.sequence(
              Commands.either(
                  PivotCmds.setAngleAndWait(() -> 35),
                  Commands.none(),
                  () -> RobotContainer.pivot.getCurrentAngle() < -120),
              ElevatorCmds.setHeightAndWait(SSConstants.Elevator.L3_HEIGHT_IN),
              Commands.parallel(
                  PivotCmds.setAngle(SSConstants.Pivot.L3_ANGLE_DEG),
                  ShoulderCmds.setAngleAndWait(SSConstants.Shoulder.L3_ANGLE_DEG)),
              PivotCmds.waitUntilAtTarget())),
  L4_PREP(
      () ->
          Commands.sequence(
              Commands.either(
                  PivotCmds.setAngleAndWait(() -> 35),
                  Commands.none(),
                  () -> RobotContainer.pivot.getCurrentAngle() < -120),
              Commands.parallel(
                  ElevatorCmds.setHeightAndWait(SSConstants.Elevator.L4_HEIGHT_IN),
                  PivotCmds.setAngle(SSConstants.Pivot.L4_ANGLE_DEG),
                  ShoulderCmds.setAngleAndWait(SSConstants.Shoulder.L4_ANGLE_DEG)),
              PivotCmds.waitUntilAtTarget())),
  CORAL_SCORE(
      () ->
          Commands.sequence(
              RollerCmds.setEnableLimitSwitch(false),
              RollerCmds.setSpeed(SSConstants.Roller.L2_PLUS_CORAL_SCORE_SPEED))),
  ALGAE_GRAB(() -> AlgaeClawCmds.setSpeed(SSConstants.AlgaeClaw.ALGAE_GRAB_SPEED)),

  ALGAE_GRAB_AND_CORAL_SCORE(
      () ->
          Commands.sequence(
              ALGAE_GRAB.getCommand(), AlgaeClawCmds.waitUntilAlgae(2), CORAL_SCORE.getCommand())),
  ALGAE_SAFE_RETRACT(
      () -> Commands.sequence(PivotCmds.setAngle(SSConstants.Pivot.SAFE_ANGLE_DEGS))),

  PROCESSOR_PREP(
      () ->
          Commands.sequence(
              ElevatorCmds.setHeightAndWait(SSConstants.Elevator.PROCESSOR_PREP_HEIGHT_IN),
              ShoulderCmds.setAngleAndWait(SSConstants.Shoulder.PROCESSOR_SCORE_ANGLE_DEG),
              PivotCmds.setAngleAndWait(SSConstants.Pivot.PROCESSOR_SCORE_ANGLE_DEG),
              ElevatorCmds.setHeightAndWait(SSConstants.Elevator.PROCESSOR_SCORE_HEIGHT_IN))),
  PROCESSOR_SCORE(() -> Commands.sequence(PROCESSOR_PREP.getCommand())),
  CLIMB(
      () ->
          Commands.sequence(
              ShoulderCmds.setAngle(SSConstants.Shoulder.CLIMB_ANGLE_DEGS),
              PivotCmds.setAngle(SSConstants.Pivot.CLIMB_ANGLE_DEGS))),
  CLIMB_PREP(
      () ->
          Commands.sequence(
              // move the arm out of the way
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
          Commands.sequence(
              ShoulderCmds.setAngle(SSConstants.Shoulder.BARGE_ANGLE_DEGREES),
              PivotCmds.setAngle(SSConstants.Pivot.BARGE_ANGLE_DEG),
              ElevatorCmds.setHeightAndWait(SSConstants.Elevator.BARGE_HEIGHT_IN),
              ShoulderCmds.waitUntilAtTarget(),
              PivotCmds.waitUntilAtTarget()));

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
