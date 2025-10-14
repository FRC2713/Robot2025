package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class ArmCmds {

  public static Command armSetAngle(double targetAngle) {
    return armSetAngle(() -> targetAngle);
  }

  public static Command armSetAngle(DoubleSupplier targetAngle) {
    return new InstantCommand(() -> RobotContainer.arm.setTargetAngle(targetAngle.getAsDouble()));
  }

  public static Command armWaitUntilAtTarget() {
    return new WaitUntilCommand(() -> RobotContainer.arm.isAtTarget());
  }

  public static Command armSetAngleAndWait(double targetAngle) {
    return armSetAngleAndWait(() -> targetAngle);
  }

  public static Command armSetAngleAndWait(DoubleSupplier targetAngle) {
    return Commands.sequence(armSetAngle(targetAngle), handWaitUntilAtTarget());
  }

  public static Command handSetSpeed(DoubleSupplier targetRPM) {
    return new InstantCommand(() -> RobotContainer.arm.setCoralRPM(targetRPM.getAsDouble()));
  }

  public static Command handWaitUntilAtTarget() {
    return new WaitUntilCommand(() -> RobotContainer.arm.handIsCoralAtTarget());
  }

  public static Command handWaitUntilCoral() {
    return handWaitUntilCoral(Integer.MAX_VALUE);
  }

  public static Command handWaitUntilNoCoral() {
    return handWaitUntilNoCoral(Integer.MAX_VALUE);
  }

  public static Command handWaitUntilCoral(double timeout) {
    return Commands.race(
        Commands.waitSeconds(timeout),
        new WaitUntilCommand(() -> RobotContainer.arm.handHasCoral()));
  }

  public static Command handWaitUntilNoCoral(double timeout) {
    return Commands.race(
        Commands.waitSeconds(timeout),
        new WaitUntilCommand(() -> !RobotContainer.arm.handHasCoral()));
  }

  public static Command handSetEnableLimitSwitch(boolean setEnable) {
    return new InstantCommand(() -> RobotContainer.arm.handSetEnableLimitSwitch(setEnable));
  }

  public static Command handSetEnableLimitSwitch(BooleanSupplier setEnable) {
    return new InstantCommand(
        () -> RobotContainer.arm.handSetEnableLimitSwitch(setEnable.getAsBoolean()));
  }

  public static Command handSetSpeedAndWait(DoubleSupplier targetRPM) {
    return Commands.sequence(handSetSpeed(targetRPM), handWaitUntilAtTarget());
  }

  public static Command setSpeedAndWaitForNoCoral(DoubleSupplier targetRPM) {
    return Commands.sequence(
        handSetEnableLimitSwitch(false),
        handSetSpeed(targetRPM),
        handWaitUntilNoCoral(0.5),
        handSetEnableLimitSwitch(true));
  }

  // todo: finish labling with hand and arms
  public static Command driveUntilLimitSet(DoubleSupplier targetRPM) {
    return Commands.sequence(
        handSetSpeed(targetRPM), handWaitUntilCoral(), handSetSpeed(() -> 0.0));
  }

  public static Command score(DoubleSupplier targetRpm) {
    return Commands.sequence(handSetEnableLimitSwitch(false), handSetSpeed(targetRpm));
  }

  public static Command intake(DoubleSupplier targetRpm) {
    return Commands.sequence(handSetEnableLimitSwitch(true), handSetSpeed(targetRpm));
  }
}
