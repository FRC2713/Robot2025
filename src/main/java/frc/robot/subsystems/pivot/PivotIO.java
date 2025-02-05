package frc.robot.subsystems.pivot;

import frc.robot.util.LoggedTunablePID;
import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {

  @AutoLog
  public static class PivotInputs {
    public double velocityDPS = 0.0;
    public double voltage = 0.0;
    public double angleDegrees = 0.0;

    public double commandedAngleDegs = 0.0;
  }

  public default void updateInputs(PivotInputs inputs) {}
  ;

  public default void setVoltage(double volts) {}
  ;

  public default void setTargetAngle(double degrees) {}
  ;

  public default void setPID(LoggedTunablePID pid) {}
  ;
}
