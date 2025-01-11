package frc.robot.subsystems;


import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pivot extends SubsystemBase {
  private final PivotInputsAutoLogged inputs;
  private final PivotIO IO;

  public Pivot(PivotIO IO) {
    this.inputs = new PivotInputsAutoLogged();
    IO.updateInputs(inputs);
    this.IO = IO;
  }

  public void periodic() {
    IO.updateInputs(inputs);
    Logger.processInputs("Pivot", inputs);

  }
  public void setTargetAngle(double degrees) {
   
  }
}
