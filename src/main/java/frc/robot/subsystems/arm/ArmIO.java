package frc.robot.subsystems.arm;

import frc.robot.util.LoggedTunableGains;
import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
  @AutoLog
  public static class ArmInputs {
    public double velocityDPS = 0.0;
    public double armVoltage = 0.0;
    public double angleDegrees = 0.0;
    public double absoluteAngleDegrees = 0.0;
    public double setpointVelocity = 0.0;
    public double commandedAngleDegs = 0.0;

    public double handVoltage = 0.0;
    public double handRPM = 0.0;
    public double handCommandedRPM = 0.0;
    public boolean hasAlgae = false;
    public boolean hasCoral = false;
    public int currentLimit = 0;
  }

  public default void handSetEnableLimitSwitch(boolean enabled) {}
  ;

  public default void updateInputs(ArmInputs inputs) {}
  ;

  public default void armSetVoltage(double volts) {}
  ;

  public default void handSetVoltage(double volts) {}
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
