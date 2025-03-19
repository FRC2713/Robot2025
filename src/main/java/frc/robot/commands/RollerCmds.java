package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import java.util.function.DoubleSupplier;

public class RollerCmds {
  public static Command setSpeed(DoubleSupplier targetRPM) {
    return new InstantCommand(() -> RobotContainer.rollers.setRPM(targetRPM.getAsDouble()));
  }

  public static Command waitUntilAtTarget() {
    return new WaitUntilCommand(() -> RobotContainer.rollers.isAtTarget());
  }

  public static Command waitUntilCoral() {
    return waitUntilCoral(Integer.MAX_VALUE);
  }

  public static Command waitUntilNoCoral() {
    return waitUntilNoCoral(Integer.MAX_VALUE);
  }

  public static Command waitUntilCoral(double timeout) {
    return Commands.race(
        Commands.waitSeconds(timeout),
        new WaitUntilCommand(() -> RobotContainer.rollers.hasCoral()));
  }

  public static Command waitUntilNoCoral(double timeout) {
    return Commands.race(
        Commands.waitSeconds(timeout),
        new WaitUntilCommand(() -> !RobotContainer.rollers.hasCoral()));
  }

  public static Command setEnableLimitSwitch(boolean setEnable) {
    return new InstantCommand(() -> RobotContainer.rollers.setEnableLimitSwitch(setEnable));
  }

  public static Command setSpeedAndWait(DoubleSupplier targetRPM) {
    return Commands.sequence(setSpeed(targetRPM), waitUntilAtTarget());
  }

  public static Command setSpeedAndWaitForNoCoral(DoubleSupplier targetRPM) {
    return Commands.sequence(
        setEnableLimitSwitch(false),
        setSpeed(targetRPM),
        waitUntilNoCoral(0.5),
        setEnableLimitSwitch(true));
  }

  public static Command driveUntilLimitSet(DoubleSupplier targetRPM) {
    return Commands.sequence(setSpeed(targetRPM), waitUntilCoral(), setSpeed(() -> 0.0));
  }

  public static Command score(DoubleSupplier targetRpm) {
    return Commands.sequence(setEnableLimitSwitch(false), setSpeed(targetRpm));
  }

  public static Command intake(DoubleSupplier targetRpm) {
    return Commands.sequence(setEnableLimitSwitch(true), setSpeed(targetRpm));
  }
}
