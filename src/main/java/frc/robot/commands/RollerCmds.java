package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;

public class RollerCmds {

  public static Command setAlgaeSpeed(double targetRPM) {

    return new ConditionalCommand(
        new InstantCommand(() -> RobotContainer.rollers.setAlgaeRPM(0)),
        new InstantCommand(() -> RobotContainer.rollers.setAlgaeRPM(targetRPM)),
        () -> RobotContainer.rollers.hasCoral());
  }

  public static Command setTubeSpeed(double targetRPM) {
    return new ConditionalCommand(
        new InstantCommand(() -> RobotContainer.rollers.setTubeRPM(0)),
        new InstantCommand(() -> RobotContainer.rollers.setTubeRPM(targetRPM)),
        () -> RobotContainer.rollers.hasCoral());
  }

  public static Command waitUntilAlgaeAtTarget() {
    return new WaitUntilCommand(() -> RobotContainer.rollers.isAlgaeAtTarget());
  }

  public static Command waitUntilTubeAtTarget() {
    return new WaitUntilCommand(() -> RobotContainer.rollers.isTubeAtTarget());
  }

  public static Command setAlgaeSpeedAndWait(double targetRPM) {
    return Commands.sequence(setAlgaeSpeed(targetRPM), waitUntilAlgaeAtTarget());
  }

  public static Command setTubeSpeedAndWait(double targetRPM) {
    return Commands.sequence(setTubeSpeed(targetRPM), waitUntilTubeAtTarget());
  }
}
