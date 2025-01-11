package frc.robot.commands.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.pivot.Pivot;

public class SetPivotAngleCommand extends Command {
  public double driverInput;
  public Pivot pivotThing;

  public SetPivotAngleCommand(Pivot PivotThing, double driverInput) {
    this.driverInput = driverInput;
    this.pivotThing = PivotThing;
  }

  public void initialize() {
    this.pivotThing.setTargetAngle(driverInput);
  }

  public void execute() {}

  public boolean isFinished() {
    return true;
  }

  public void end(boolean isInterupted) {}
}
