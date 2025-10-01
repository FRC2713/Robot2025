package frc.robot.subsystems.arm;

import frc.robot.util.LoggedTunableGains;
import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
  @AutoLog
  public static class ArmInputs {
    public double velocityDPS = 0.0;
    public double voltage = 0.0;
    public double angleDegrees = 0.0;
    public double targetAngle = 0.0;
    public double commandedAngleDegs = 0.0;
  }

  public default void updateInputs(ArmInputs inputs) {}
  ;

  public default void setVoltage(double volts) {}
  ;

  public default void setTargetAngle(double degrees) {}
  ;

  public default void setPID(LoggedTunableGains pid) {}
  ;

  public default void setBus(double bus) {}
  ;

  public default boolean isAtTarget() {
    return true;
  }
}
