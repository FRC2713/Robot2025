package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;

public class ElevatorCmds {
  public Command setHeightWaitCmd(double h) {
    Command cmd1 = new InstantCommand(() -> RobotContainer.elevator.setTargetHeight(h));
    Command cmd2 = new WaitUntilCommand(() -> RobotContainer.elevator.isAtTarget());
    return Commands.parallel(cmd1, cmd2);
  }

  public Command setHeightCmd(double h) {
    return new InstantCommand(() -> RobotContainer.elevator.setTargetHeight(h));
  }
}
