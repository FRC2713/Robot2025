package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.ElevatorIOSparks;

public class ElevatorHeightCommand extends Command {
  private ElevatorIOSparks elevator;
  private double height;

  public void ElevatorHeightCommand(ElevatorIOSparks elevator, double height) {
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
