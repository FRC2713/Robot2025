package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;

public class PivotCmds {

  public static Command setAngle(double targetAngle) {
    return new InstantCommand(() -> RobotContainer.pivot.setTargetAngle(targetAngle));
  }

  public static Command waitUntilAtTarget() {
    return new WaitUntilCommand(() -> RobotContainer.pivot.isAtTarget());
  }

  public static Command setAngleAndWait(double targetAngle) {
    return Commands.sequence(setAngle(targetAngle), waitUntilAtTarget());
  }
}
