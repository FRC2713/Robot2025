package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import java.util.function.DoubleSupplier;

public class AlgaeClawCmds {
  public static Command setSpeed(DoubleSupplier targetRPM) {
    return new InstantCommand(
        () -> RobotContainer.endEffector.setAlgaeRPM(targetRPM.getAsDouble()),
        RobotContainer.endEffector);
  }

  public static Command setSpeedIfNoAlgae(DoubleSupplier targetRPM) {
    return Commands.either(
        Commands.none(), setSpeed(targetRPM), RobotContainer.endEffector::hasAlgae);
  }

  public static Command waitUntilAtTarget() {
    return new WaitUntilCommand(() -> RobotContainer.endEffector.isAlgaeAtTarget());
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
        new WaitUntilCommand(() -> RobotContainer.endEffector.hasAlgae()));
  }

  public static Command waitUntilNoAlgae(double timeout) {
    return Commands.race(
        Commands.waitSeconds(timeout),
        new WaitUntilCommand(() -> !RobotContainer.endEffector.hasAlgae()));
  }

  // public static Command setEnableLimitSwitch(boolean setEnable) {
  //   return new InstantCommand(() -> RobotContainer.endEffector.setEnableLimitSwitch(setEnable));
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

  public static Command stop() {
    return new InstantCommand(() -> RobotContainer.endEffector.setAlgaeRPM(0.0));
  }

  public static Command setAlgaeCurrentLimit(int algaeCurrentLimit) {
    return new InstantCommand(
        () -> RobotContainer.endEffector.setAlgaeCurrentLimit(algaeCurrentLimit));
  }
}
