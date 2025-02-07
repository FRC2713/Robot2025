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
    return new WaitUntilCommand(() -> RobotContainer.rollers.hasCoral());
  }

  public static Command waitUntilNoCoral() {
    return new WaitUntilCommand(() -> !RobotContainer.rollers.hasCoral());
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
        waitUntilNoCoral(),
        setEnableLimitSwitch(true));
  }
}
