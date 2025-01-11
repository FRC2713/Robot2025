package frc.robot.commands.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;

public class ElevatorCmds {

  public static Command setHeight(double targetHeight) {
    return new SetElevatorHeightCommand(RobotContainer.elevator, targetHeight);
  }

  public static Command waitUntilAtTarget() {
    return new WaitUntilCommand(() -> RobotContainer.elevator.isAtTarget());
  }

  public static Command setHeightAndWait(double targetHeight) {
    return Commands.sequence(setHeight(targetHeight), waitUntilAtTarget());
  }
}
