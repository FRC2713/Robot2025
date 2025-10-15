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
    return Commands.sequence(armSetAngle(targetAngle));
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

  public static Command handSetVoltage(int voltage) {
    return new InstantCommand(() -> RobotContainer.arm.handSetVoltage(voltage));
  }

}
