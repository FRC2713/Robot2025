package frc.robot.subsystems.pivot;

import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {

  @AutoLog
  public static class PivotInputs { // PascalCase
    public boolean isOn = false;
    public double velocityDPS = 0.0; // camelCase
    public double voltage = 0.0;
    public double angleDegrees = 0.0;

    public double commandedAngle = 0.0;
  }

  public default void updateInputs(PivotInputs inputs) {}
  ;

  public default void setVoltage(double volts) {}
  ;

  public default void setTargetAngle(double degrees) {}
  ;
}
