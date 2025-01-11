package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {

  @AutoLog
  public static class PivotInputs { // PascalCase
    public boolean isOn = false;
    public double velocityDPS = 0.0; // camelCase
    public double voltage = 0.0;
    public double angleDegrees = 0.0;
  }

  public void updateInputs(PivotInputs inputs);

  public void setVoltage(double Volts);
  public void setTargetAngle(double degrees);
}
