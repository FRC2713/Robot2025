package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import java.util.function.DoubleSupplier;

public class ElevatorCmds {
  public static Command setHeightAndWait(double h) {
    return setHeightAndWait(() -> h);
  }

  public static Command setHeightAndWait(DoubleSupplier h) {
    Command cmd1 =
        new InstantCommand(() -> RobotContainer.elevator.setTargetHeight(h.getAsDouble()));
    Command cmd2 = new WaitUntilCommand(() -> RobotContainer.elevator.isAtTarget());
    return Commands.parallel(cmd1, cmd2);
  }

  public static Command setHeight(double h) {
    return setHeight(() -> h);
  }

  public static Command setHeight(DoubleSupplier h) {
    return new InstantCommand(() -> RobotContainer.elevator.setTargetHeight(h.getAsDouble()));
  }

  public static Command waitUntilAtTarget() {
    return new WaitUntilCommand(() -> RobotContainer.elevator.isAtTarget());
  }
}
