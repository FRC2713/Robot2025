package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class PivotCmds {

  public static Command setAngle(double targetAngle) {
    // return new InstantCommand(() -> RobotContainer.pivotThing.setTargetAngle(targetAngle));
    return Commands.none();
  }

  public static Command waitUntilAtTarget() {
    // return new WaitUntilCommand(() -> RobotContainer.pivotThing.isAtTarget());
    return Commands.none();
  }

  public static Command setAngleAndWait(double targetAngle) {
    // return Commands.sequence(setAngle(targetAngle), waitUntilAtTarget());
    return Commands.none();
  }
}
