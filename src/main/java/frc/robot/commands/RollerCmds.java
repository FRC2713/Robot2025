package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import java.util.function.DoubleSupplier;

public class RollerCmds {
  public static Command setTubeSpeed(DoubleSupplier targetRPM) {
    return new InstantCommand(() -> RobotContainer.rollers.setTubeRPM(targetRPM.getAsDouble()));
  }

  public static Command waitUntilTubeAtTarget() {
    return new WaitUntilCommand(() -> RobotContainer.rollers.isTubeAtTarget());
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

  public static Command waitUntilAlgae(double timeout) {
    return Commands.race(
        Commands.waitSeconds(timeout),
        new WaitUntilCommand(() -> RobotContainer.rollers.hasAlgae()));
  }

  public static Command waitUntilNoCoral(double timeout) {
    return Commands.race(
        Commands.waitSeconds(timeout),
        new WaitUntilCommand(() -> !RobotContainer.rollers.hasCoral()));
  }

  public static Command setEnableLimitSwitch(boolean setEnable) {
    return new InstantCommand(() -> RobotContainer.rollers.setEnableLimitSwitch(setEnable));
  }

  public static Command setTubeSpeedAndWait(DoubleSupplier targetRPM) {
    return Commands.sequence(setTubeSpeed(targetRPM), waitUntilTubeAtTarget());
  }

  public static Command setTubeSpeedAndWaitForNoCoral(DoubleSupplier targetRPM) {
    return Commands.sequence(
        setEnableLimitSwitch(false),
        setTubeSpeed(targetRPM),
        waitUntilNoCoral(2),
        Commands.waitSeconds(1),
        setEnableLimitSwitch(true));
  }

  public static Command driveUntilLimitSet(DoubleSupplier targetRPM) {
    // () -> 0
    return Commands.sequence(setTubeSpeed(targetRPM), waitUntilCoral(), setTubeSpeed(() -> 0.0));
  }

  public static Command setEnableAlgaeLS(boolean setEnable) {
    return new InstantCommand(() -> RobotContainer.rollers.setEnableAlgaeLimitSwitch(setEnable));
  }
}
