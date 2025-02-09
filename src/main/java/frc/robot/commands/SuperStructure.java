package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.SSConstants;
import frc.robot.subsystems.constants.PivotConstants;
import java.util.function.Supplier;

public enum SuperStructure {
  // Loc_GamePiece_Action
  STARTING_CONF(
      () ->
          Commands.sequence(
              ElevatorCmds.setHeight(0),
              PivotCmds.setAngle(Units.radiansToDegrees(PivotConstants.kInitialAngleRad)),
              RollerCmds.setEnableLimitSwitch(true),
              RollerCmds.setTubeSpeedAndWait(() -> 0),
              ElevatorCmds.waitUntilAtTarget(),
              RollerCmds.waitUntilTubeAtTarget())),
  SOURCE_CORAL_INTAKE(
      () ->
          Commands.sequence(
              Commands.parallel(
                  RollerCmds.setEnableLimitSwitch(true),
                  ElevatorCmds.setHeight(SSConstants.Elevator.SOURCE_CORAL_INTAKE_HEIGHT_IN),
                  RollerCmds.setTubeSpeed(SSConstants.Roller.SOURCE_CORAL_INTAKE_SPEED),
                  PivotCmds.setAngle(SSConstants.Pivot.SOURCE_CORAL_INTAKE_ANGLE_DEG)),
              RollerCmds.waitUntilCoral(2))),
  L3_ALGAE_GRAB(
      () ->
          Commands.sequence(
              Commands.parallel(
                  ElevatorCmds.setHeight(SSConstants.Elevator.L3_ALGAE_GRAB_HEIGHT_IN),
                  RollerCmds.setTubeSpeed(SSConstants.Roller.L3_ALGAE_GRAB_SPEED),
                  PivotCmds.setAngle(SSConstants.Pivot.L3_ALGAE_GRAB_DEG)),
              RollerCmds.waitUntilAlgae(2))),
  L1_CORAL_PREP(
      () ->
          Commands.parallel(
              ElevatorCmds.setHeight(SSConstants.Elevator.L1_CORAL_PREP_HEIGHT_IN),
              PivotCmds.setAngle(SSConstants.Pivot.L1_CORAL_PREP_ANGLE_DEG))),
  L3_CORAL_PREP(
      () ->
          Commands.parallel(
              ElevatorCmds.setHeight(SSConstants.Elevator.L3_CORAL_PREP_HEIGHT_IN),
              PivotCmds.setAngle(SSConstants.Pivot.L3_CORAL_PREP_ANGLE_DEG))),
  L1_CORAL_SCORE(
      () ->
          Commands.sequence(
              Commands.parallel(
                  ElevatorCmds.setHeightAndWait(SSConstants.Elevator.L1_CORAL_SCORE_HEIGHT_IN),
                  PivotCmds.setAngle(SSConstants.Pivot.L1_CORAL_SCORE_ANGLE_DEG)),
              RollerCmds.setTubeSpeedAndWaitForNoCoral(SSConstants.Roller.L1_CORAL_SCORE_SPEED))),
  L2_CORAL_PREP(
      () ->
          Commands.parallel(
              ElevatorCmds.setHeight(SSConstants.Elevator.L2_CORAL_PREP_HEIGHT_IN),
              PivotCmds.setAngle(SSConstants.Pivot.L2_CORAL_PREP_ANGLE_DEG))),
  L3_CORAL_SCORE(
      () ->
          Commands.sequence(
              ElevatorCmds.setHeightAndWait(SSConstants.Elevator.L3_CORAL_SCORE_HEIGHT_IN),
              PivotCmds.setAngleAndWait(SSConstants.Pivot.L3_CORAL_SCORE_ANGLE_DEG),
              RollerCmds.setTubeSpeedAndWaitForNoCoral(SSConstants.Roller.L3_CORAL_SCORE_SPEED))),
  L2_CORAL_SCORE(
      () ->
          Commands.sequence(
              ElevatorCmds.setHeightAndWait(SSConstants.Elevator.L2_CORAL_SCORE_HEIGHT_IN),
              PivotCmds.setAngleAndWait(SSConstants.Pivot.L2_CORAL_SCORE_ANGLE_DEG),
              RollerCmds.setTubeSpeedAndWaitForNoCoral(SSConstants.Roller.L2_CORAL_SCORE_SPEED))),
  PROCESSOR_SCORE(
      () ->
          Commands.sequence(
              ElevatorCmds.setHeightAndWait(SSConstants.Elevator.PROCESSOR_SCORE_HEIGHT_IN),
              PivotCmds.setAngleAndWait(SSConstants.Pivot.PROCESSOR_SCORE_ANGLE_DEG),
              RollerCmds.setTubeSpeedAndWaitForNoCoral(SSConstants.Roller.PROCESSOR_SCORE_SPEED))),
  ;

  private Supplier<Command> cmd;

  private SuperStructure(Supplier<Command> cmd) {
    this.cmd = cmd;
  }

  public Command getCommand() {
    return Commands.sequence(Commands.print("Starting SS: " + this.toString()), cmd.get());
  }
}
