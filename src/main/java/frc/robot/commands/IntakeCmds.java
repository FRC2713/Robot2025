package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class IntakeCmds {
  // rollers
  public static Command setVolts(DoubleSupplier targetVolts) {
    return new InstantCommand(
        () -> RobotContainer.intake.setRollerVoltage(targetVolts.getAsDouble()));
  }

  public static Command setVoltsUntilCoral(DoubleSupplier targetVolts) {
    return Commands.sequence(
        new InstantCommand(() -> RobotContainer.intake.setRollerVoltage(targetVolts.getAsDouble())),
        new WaitUntilCommand(RobotContainer.intake::hasCoral)
        //new InstantCommand(() -> RobotContainer.intake.setRollerVoltage(0))
        );
  }

  public static Command setVolts(double targetVolts) {
    return new InstantCommand(() -> RobotContainer.intake.setRollerVoltage(targetVolts));
  }

  public static Command enableLimitSwitch(BooleanSupplier setEnable) {
    return new InstantCommand(
        () -> RobotContainer.endEffector.setEnableLimitSwitch(setEnable.getAsBoolean()));
  }

  // intake pivot
  public static Command setAngle(double targetAngle) {
    return new InstantCommand(() -> RobotContainer.intake.setTargetAngle(targetAngle));
  }

  public static Command setAngle(DoubleSupplier targetAngle) {
    return new InstantCommand(
        () -> RobotContainer.intake.setTargetAngle(targetAngle.getAsDouble()));
  }

  public static Command waitUntilAtTarget() {
    return new WaitUntilCommand(() -> RobotContainer.intake.intakePivotIsAtTarget());
  }

  public static Command setAngleAndWait(double targetAngle) {
    return Commands.sequence(IntakeCmds.setAngle(targetAngle), IntakeCmds.waitUntilAtTarget());
  }

  public static Command setAngleAndWait(DoubleSupplier targetAngle) {
    return Commands.sequence(IntakeCmds.setAngle(targetAngle), IntakeCmds.waitUntilAtTarget());
  }
}
