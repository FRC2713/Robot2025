package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.SSConstants;
import frc.robot.subsystems.constants.PivotConstants;
import java.util.function.Supplier;

public enum SuperStructure {
  STARTING_CONF(
      () ->
          Commands.sequence(
              ElevatorCmds.setHeight(0),
              PivotCmds.setAngle(Units.radiansToDegrees(PivotConstants.kInitialAngleRad)),
              RollerCmds.setEnableLimitSwitch(true),
              RollerCmds.setTubeSpeedAndWait(() -> 0),
              ElevatorCmds.waitUntilAtTarget(),
              RollerCmds.waitUntilTubeAtTarget())),
  L1_CORAL_PREP(
      () ->
          Commands.parallel(
              ElevatorCmds.setHeight(SSConstants.Elevator.L1_CORAL_PREP_HEIGHT_IN),
              PivotCmds.setAngle(SSConstants.Pivot.L1_CORAL_PREP_ANGLE_DEG))),
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
  L2_CORAL_SCORE(
      () ->
          Commands.sequence(
              Commands.parallel(
                  ElevatorCmds.setHeightAndWait(SSConstants.Elevator.L2_CORAL_SCORE_HEIGHT_IN),
                  PivotCmds.setAngleAndWait(SSConstants.Pivot.L2_CORAL_SCORE_ANGLE_DEG)),
              RollerCmds.setTubeSpeedAndWaitForNoCoral(SSConstants.Roller.L2_CORAL_SCORE_SPEED)));

  private Supplier<Command> cmd;

  private SuperStructure(Supplier<Command> cmd) {
    this.cmd = cmd;
  }

  public Command getCommand() {
    return cmd.get();
  }
}
