package frc.robot.commands.subsystemCmds;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import java.util.function.DoubleSupplier;

public class AlgaeClawCmds {
  public static Command setSpeed(DoubleSupplier targetRPM) {
    return new InstantCommand(() -> RobotContainer.algaeClaw.setRPM(targetRPM.getAsDouble()));
  }

  public static Command setSpeedIfNoAlgae(DoubleSupplier targetRPM) {
    return Commands.either(
        Commands.none(), setSpeed(targetRPM), RobotContainer.algaeClaw::hasAlgae);
  }

  public static Command waitUntilAtTarget() {
    return new WaitUntilCommand(() -> RobotContainer.algaeClaw.isAtTarget());
  }

  public static Command waitUntilAlgae() {
    return waitUntilAlgae(Integer.MAX_VALUE);
  }

  public static Command waitUntilNoAlgae() {
    return waitUntilNoAlgae(Integer.MAX_VALUE);
  }

  public static Command waitUntilAlgae(double timeout) {
    return Commands.race(
        Commands.waitSeconds(timeout),
        new WaitUntilCommand(() -> RobotContainer.algaeClaw.hasAlgae()));
  }

  public static Command waitUntilNoAlgae(double timeout) {
    return Commands.race(
        Commands.waitSeconds(timeout),
        new WaitUntilCommand(() -> !RobotContainer.algaeClaw.hasAlgae()));
  }

  // public static Command setEnableLimitSwitch(boolean setEnable) {
  //   return new InstantCommand(() -> RobotContainer.algaeClaw.setEnableLimitSwitch(setEnable));
  // }

  public static Command setSpeedAndWait(DoubleSupplier targetRPM) {
    return Commands.sequence(setSpeed(targetRPM), waitUntilAtTarget());
  }

  public static Command setSpeedAndWaitForNoAlgae(DoubleSupplier targetRPM) {
    return Commands.sequence(setSpeed(targetRPM), waitUntilNoAlgae(0.5), setSpeed(() -> 0));
  }

  public static Command driveUntilLimitSet(DoubleSupplier targetRPM) {
    // () -> 0
    return Commands.sequence(setSpeed(targetRPM), waitUntilAlgae(), setSpeed(() -> 0.0));
  }

  public static Command score(DoubleSupplier targetRpm) {
    return Commands.sequence(setSpeed(targetRpm));
  }

  public static Command intake(DoubleSupplier targetRpm) {
    return Commands.sequence(setSpeed(targetRpm));
  }
}
