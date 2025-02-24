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
              AlgaeClawCmds.setEnableLimitSwitch(true),
              ElevatorCmds.waitUntilAtTarget())),
  SOURCE_CORAL_INTAKE(
      () ->
          Commands.sequence(
              Commands.parallel(
                  RollerCmds.setEnableLimitSwitch(true),
                  RollerCmds.setEnableAlgaeLS(false),
                  ElevatorCmds.setHeight(SSConstants.Elevator.SOURCE_CORAL_INTAKE_HEIGHT_IN),
                  RollerCmds.setSpeed(SSConstants.Roller.SOURCE_CORAL_INTAKE_SPEED),
                  PivotCmds.setAngle(SSConstants.Pivot.SOURCE_CORAL_INTAKE_ANGLE_DEG),
                  ShoulderCmds.setAngle(SSConstants.Shoulder.SOURCE_CORAL_INTAKE_ANGLE_DEG)),
              RollerCmds.waitUntilCoral(2))),
  L1_ALGAE_GRAB(
      () ->
          Commands.sequence(
              AlgaeClawCmds.setEnableLimitSwitch(false),
              Commands.parallel(
                  ElevatorCmds.setHeightAndWait(SSConstants.Elevator.L1_ALGAE_GRAB_HEIGHT_IN),
                  ShoulderCmds.setAngleAndWait(SSConstants.Shoulder.L1_ALGAE_GRAB_ANGLE_DEG),
                  AlgaeClawCmds.setSpeed(SSConstants.AlgaeClaw.L1_ALGAE_GRAB_SPEED)),
              PivotCmds.setAngle(SSConstants.Pivot.L1_ALGAE_GRAB_DEG),
              Commands.waitSeconds(0.1),
              AlgaeClawCmds.setEnableLimitSwitch(true),
              AlgaeClawCmds.waitUntilAlgae(2))),
  L3_ALGAE_GRAB(
      () ->
          Commands.sequence(
              Commands.parallel(
                  ElevatorCmds.setHeight(SSConstants.Elevator.L3_ALGAE_GRAB_HEIGHT_IN),
                  RollerCmds.setSpeed(SSConstants.AlgaeClaw.L3_ALGAE_GRAB_SPEED),
                  PivotCmds.setAngle(SSConstants.Pivot.L3_ALGAE_GRAB_DEG),
                  ShoulderCmds.setAngle(SSConstants.Shoulder.L3_ALGAE_GRAB_ANGLE_DEG)),
              AlgaeClawCmds.waitUntilAlgae(2))),
  L1_CORAL_PREP(
      () ->
          Commands.parallel(
              ElevatorCmds.setHeight(SSConstants.Elevator.L1_CORAL_PREP_HEIGHT_IN),
              PivotCmds.setAngle(SSConstants.Pivot.L1_CORAL_PREP_ANGLE_DEG),
              ShoulderCmds.setAngle(SSConstants.Shoulder.L1_CORAL_PREP_ANGLE_DEG))),
  L2_CORAL_PREP(
      () ->
          Commands.parallel(
              ElevatorCmds.setHeight(SSConstants.Elevator.L2_CORAL_PREP_HEIGHT_IN),
              ShoulderCmds.setAngle(SSConstants.Shoulder.L2_CORAL_PREP_ANGLE_DEG),
              PivotCmds.setAngle(SSConstants.Pivot.L2_CORAL_PREP_ANGLE_DEG))),
  L3_CORAL_PREP(
      () ->
          Commands.parallel(
              ElevatorCmds.setHeight(SSConstants.Elevator.L3_CORAL_PREP_HEIGHT_IN),
              ShoulderCmds.setAngle(SSConstants.Shoulder.L3_CORAL_PREP_ANGLE_DEG),
              PivotCmds.setAngle(SSConstants.Pivot.L3_CORAL_PREP_ANGLE_DEG))),
  L4_CORAL_PREP(
      () ->
          Commands.parallel(
              ElevatorCmds.setHeight(SSConstants.Elevator.L4_CORAL_PREP_HEIGHT_IN),
              ShoulderCmds.setAngle(SSConstants.Shoulder.L4_CORAL_PREP_ANGLE_DEG),
              PivotCmds.setAngle(SSConstants.Pivot.L4_CORAL_PREP_ANGLE_DEG))),
  L1_CORAL_SCORE(
      () ->
          Commands.sequence(
              Commands.parallel(
                  ElevatorCmds.setHeightAndWait(SSConstants.Elevator.L1_CORAL_SCORE_HEIGHT_IN),
                  ShoulderCmds.setAngleAndWait(SSConstants.Shoulder.L1_CORAL_SCORE_ANGLE_DEG),
                  PivotCmds.setAngleAndWait(SSConstants.Pivot.L1_CORAL_SCORE_ANGLE_DEG)),
              RollerCmds.setSpeedAndWaitForNoCoral(SSConstants.Roller.L1_CORAL_SCORE_SPEED))),
  L2_CORAL_SCORE(
      () ->
          Commands.sequence(
              ElevatorCmds.setHeightAndWait(SSConstants.Elevator.L2_CORAL_SCORE_HEIGHT_IN),
              ShoulderCmds.setAngleAndWait(SSConstants.Shoulder.L2_CORAL_SCORE_ANGLE_DEG),
              PivotCmds.setAngleAndWait(SSConstants.Pivot.L2_CORAL_SCORE_ANGLE_DEG),
              RollerCmds.setSpeedAndWaitForNoCoral(SSConstants.Roller.L2_CORAL_SCORE_SPEED))),
  L3_CORAL_SCORE(
      () ->
          Commands.sequence(
              ElevatorCmds.setHeightAndWait(SSConstants.Elevator.L3_CORAL_SCORE_HEIGHT_IN),
              ShoulderCmds.setAngleAndWait(SSConstants.Shoulder.L3_CORAL_SCORE_ANGLE_DEG),
              PivotCmds.setAngleAndWait(SSConstants.Pivot.L3_CORAL_SCORE_ANGLE_DEG),
              RollerCmds.setSpeedAndWaitForNoCoral(SSConstants.Roller.L3_CORAL_SCORE_SPEED))),
  L4_CORAL_SCORE(
      () ->
          Commands.sequence(
              ElevatorCmds.setHeightAndWait(SSConstants.Elevator.L4_CORAL_SCORE_HEIGHT_IN),
              ShoulderCmds.setAngleAndWait(SSConstants.Shoulder.L4_CORAL_SCORE_ANGLE_DEG),
              PivotCmds.setAngleAndWait(SSConstants.Pivot.L4_CORAL_SCORE_ANGLE_DEG),
              RollerCmds.setSpeedAndWaitForNoCoral(SSConstants.Roller.L4_CORAL_SCORE_SPEED))),
  PROCESSOR_SCORE(
      () ->
          Commands.sequence(
              ElevatorCmds.setHeightAndWait(SSConstants.Elevator.PROCESSOR_SCORE_HEIGHT_IN),
              ShoulderCmds.setAngleAndWait(SSConstants.Shoulder.PROCESSOR_SCORE_ANGLE_DEG),
              PivotCmds.setAngleAndWait(SSConstants.Pivot.PROCESSOR_SCORE_ANGLE_DEG),
              RollerCmds.setSpeedAndWaitForNoCoral(SSConstants.AlgaeClaw.PROCESSOR_SCORE_SPEED))),
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
}
