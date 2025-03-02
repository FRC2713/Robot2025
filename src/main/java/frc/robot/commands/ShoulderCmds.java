package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import java.util.function.DoubleSupplier;

public class ShoulderCmds {

  public static Command setAngle(double targetAngle) {
    return setAngle(() -> targetAngle);
  }

  public static Command setAngle(DoubleSupplier targetAngle) {
    return new InstantCommand(
        () -> RobotContainer.shoulder.setTargetAngle(targetAngle.getAsDouble()));
  }

  public static Command waitUntilAtTarget() {
    return new WaitUntilCommand(() -> RobotContainer.shoulder.isAtTarget());
  }

  public static Command setAngleAndWait(double targetAngle) {
    return setAngleAndWait(() -> targetAngle);
  }

  public static Command setAngleAndWait(DoubleSupplier targetAngle) {
    return Commands.sequence(setAngle(targetAngle), waitUntilAtTarget());
  }

}
