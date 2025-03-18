package frc.robot.commands.subsystemCmds;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import java.util.function.DoubleSupplier;

public class PivotCmds {

  public static Command setAngle(double targetAngle) {
    return setAngle(() -> targetAngle);
  }

  public static Command avoidSelfCollision() {
    return Commands.either(
        setAngleAndWait(35), Commands.none(), () -> RobotContainer.pivot.getCurrentAngle() < -120);
  }

  public static Command setAngle(DoubleSupplier targetAngle) {
    return new InstantCommand(() -> RobotContainer.pivot.setTargetAngle(targetAngle.getAsDouble()));
  }

  public static Command waitUntilAtTarget() {
    return new WaitUntilCommand(() -> RobotContainer.pivot.isAtTarget());
  }

  public static Command setAngleAndWait(double targetAngle) {
    return setAngleAndWait(() -> targetAngle);
  }

  public static Command setAngleAndWait(DoubleSupplier targetAngle) {
    return Commands.sequence(setAngle(targetAngle), waitUntilAtTarget());
  }
}
