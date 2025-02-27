package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.SSConstants;
import frc.robot.subsystems.constants.PivotConstants;
import frc.robot.subsystems.constants.ShoulderConstants;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public enum SuperStructure {
  // Loc_GamePiece_Action
  STARTING_CONF(
      () ->
          Commands.sequence(
              ElevatorCmds.setHeight(0),
              PivotCmds.setAngle(Units.radiansToDegrees(PivotConstants.kInitialAngleRad)),
              ShoulderCmds.setAngle(Units.radiansToDegrees(ShoulderConstants.kInitialAngleRad)),
              RollerCmds.setEnableLimitSwitch(true),
              AlgaeClawCmds.setSpeed(() -> 0),
              ElevatorCmds.waitUntilAtTarget())),
  SOURCE_CORAL_INTAKE(
      () ->
          Commands.sequence(
              Commands.parallel(
                  RollerCmds.setEnableLimitSwitch(true),
                  ElevatorCmds.setHeight(SSConstants.Elevator.SOURCE_CORAL_INTAKE_HEIGHT_IN),
                  RollerCmds.setSpeed(SSConstants.Roller.SOURCE_CORAL_INTAKE_SPEED),
                  PivotCmds.setAngle(SSConstants.Pivot.SOURCE_CORAL_INTAKE_ANGLE_DEG),
                  ShoulderCmds.setAngle(SSConstants.Shoulder.SOURCE_CORAL_INTAKE_ANGLE_DEG)),
              RollerCmds.waitUntilCoral(2))),
  L1_PREP(
      () ->
          Commands.sequence(
              ElevatorCmds.setHeightAndWait(SSConstants.Elevator.L1_CORAL_PREP_HEIGHT_IN),
              Commands.parallel(
                  PivotCmds.setAngle(SSConstants.Pivot.L1_CORAL_PREP_ANGLE_DEG),
                  ShoulderCmds.setAngleAndWait(SSConstants.Shoulder.L1_CORAL_PREP_ANGLE_DEG)),
              PivotCmds.waitUntilAtTarget())),
  L2_PREP(
      () ->
          Commands.sequence(
              ElevatorCmds.setHeightAndWait(SSConstants.Elevator.L2_CORAL_PREP_HEIGHT_IN),
              Commands.parallel(
                  PivotCmds.setAngle(SSConstants.Pivot.L2_CORAL_PREP_ANGLE_DEG),
                  ShoulderCmds.setAngleAndWait(SSConstants.Shoulder.L2_CORAL_PREP_ANGLE_DEG)),
              PivotCmds.waitUntilAtTarget())),
  L3_PREP(
      () ->
          Commands.sequence(
              ElevatorCmds.setHeightAndWait(SSConstants.Elevator.L3_CORAL_PREP_HEIGHT_IN),
              Commands.parallel(
                  PivotCmds.setAngle(SSConstants.Pivot.L3_CORAL_PREP_ANGLE_DEG),
                  ShoulderCmds.setAngleAndWait(SSConstants.Shoulder.L3_CORAL_PREP_ANGLE_DEG)),
              PivotCmds.waitUntilAtTarget())),
  L4_PREP(
      () ->
          Commands.sequence(
              ElevatorCmds.setHeightAndWait(SSConstants.Elevator.L4_CORAL_PREP_HEIGHT_IN),
              Commands.parallel(
                  PivotCmds.setAngle(SSConstants.Pivot.L4_CORAL_PREP_ANGLE_DEG),
                  ShoulderCmds.setAngleAndWait(SSConstants.Shoulder.L4_CORAL_PREP_ANGLE_DEG)),
              PivotCmds.waitUntilAtTarget())),
  CORAL_SCORE(() -> RollerCmds.setSpeed(SSConstants.Roller.L2_PLUS_CORAL_SCORE_SPEED)),
  ALGAE_GRAB(
      () -> Commands.sequence(AlgaeClawCmds.setSpeed(SSConstants.AlgaeClaw.ALGAE_GRAB_SPEED))),
  ALGAE_GRAB_AND_CORAL_SCORE(
      () ->
          Commands.sequence(
              ALGAE_GRAB.getCommand(), AlgaeClawCmds.waitUntilAlgae(2), CORAL_SCORE.getCommand())),
  PROCESSOR_PREP(
      () ->
          Commands.sequence(
              ElevatorCmds.setHeightAndWait(SSConstants.Elevator.PROCESSOR_SCORE_HEIGHT_IN),
              Commands.parallel(
                  ShoulderCmds.setAngleAndWait(SSConstants.Shoulder.PROCESSOR_SCORE_ANGLE_DEG),
                  PivotCmds.setAngleAndWait(SSConstants.Pivot.PROCESSOR_SCORE_ANGLE_DEG)))),
  PROCESSOR_SCORE(
      () ->
          Commands.sequence(
              ElevatorCmds.setHeightAndWait(SSConstants.Elevator.PROCESSOR_SCORE_HEIGHT_IN),
              ShoulderCmds.setAngleAndWait(SSConstants.Shoulder.PROCESSOR_SCORE_ANGLE_DEG),
              PivotCmds.setAngleAndWait(SSConstants.Pivot.PROCESSOR_SCORE_ANGLE_DEG),
              AlgaeClawCmds.setSpeedAndWaitForNoAlgae(
                  SSConstants.AlgaeClaw.PROCESSOR_SCORE_SPEED))),
  ;

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
