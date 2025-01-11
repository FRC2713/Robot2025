package frc.robot.commands.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;

public class SetElevatorHeightCommand extends Command {
  private Elevator elevator;
  private double height;

  public SetElevatorHeightCommand(Elevator elevator, double height) {
    this.elevator = elevator;
    this.height = height;
  }

  public void initialize() {
    this.elevator.setTargetHeight(this.height);
  }

  public void execute() {}

  public boolean isFinished() {
    return true;
  }

  public void end(boolean isInterupted) {}
}
