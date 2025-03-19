package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.SetpointConstants;
import java.util.function.DoubleSupplier;

public class ClimberCmds {

  public static Command setAngle(double targetAngle) {
    return setAngle(() -> targetAngle);
  }

  public static Command configureSoftLimits(DoubleSupplier minDeg, DoubleSupplier maxDeg) {
    return new InstantCommand(
        () ->
            RobotContainer.climber.configureSoftLimits(minDeg.getAsDouble(), maxDeg.getAsDouble()));
  }

  public static Command setServoPos(DoubleSupplier pos) {
    return new InstantCommand(() -> RobotContainer.climber.setServoPos(pos.getAsDouble()));
  }

  public static Command setVoltage(DoubleSupplier volts) {
    return new InstantCommand(
        () -> {
          RobotContainer.climber.setVoltage(volts.getAsDouble());
        });
  }

  public static Command setAngle(DoubleSupplier targetAngle) {
    return new InstantCommand(
        () -> RobotContainer.climber.setTargetAngle(targetAngle.getAsDouble()));
  }

  public static Command waitUntilAtTarget() {
    return new WaitUntilCommand(() -> RobotContainer.climber.isAtTarget());
  }

  public static Command setAngleAndWait(double targetAngle) {
    return setAngleAndWait(() -> targetAngle);
  }

  public static Command setAngleAndWait(DoubleSupplier targetAngle) {
    return Commands.sequence(setAngle(targetAngle), waitUntilAtTarget());
  }

  // apply voltage to the climber until it reaches the max angle
  // soft limits will help us here, but also set the voltage to zero when it's reached.
  public static Command deploy() {
    return Commands.sequence(
        setVoltage(() -> 10),
        Commands.waitUntil(
            () ->
                RobotContainer.climber.getCurrentAngle()
                    >= SetpointConstants.Climber.MAX_ANGLE_CLIMBING.getAsDouble()),
        setVoltage(() -> 0));
  }
}
