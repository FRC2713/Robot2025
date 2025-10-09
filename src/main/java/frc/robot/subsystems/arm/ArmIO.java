package frc.robot.subsystems.arm;

import frc.robot.util.LoggedTunableGains;
import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
  @AutoLog
  public static class ArmInputs {
    public double velocityDPS = 0.0;
    public double voltage = 0.0;
    public double angleDegrees = 0.0;
    public double absoluteAngleDegrees = 0.0;
    public double setpointVelocity = 0.0;

    public double commandedAngleDegs = 0.0;
    public double tubeOutputVoltage = 0.0;
    public double tubeCurrentAmps = 0.0;
    public double tubeVelocityRPM = 0.0;
    public double commandedTubeRPM = 0.0;
    public double tubePositionDegs = 0.0;
    public double algaeRollersOutputVoltage = 0.0;
    public double algaeRollersCurrentAmps = 0.0;
    public double algaeRollersVelocityRPM = 0.0;
    public double commandedAlgaeRollersRPM = 0.0;
    public double algaeRollersPositionDegs = 0.0;
    public boolean hasAlgae = false;
    public boolean hasCoral = false;
    public double algaeSensorDistance = 0.0;
    public int currentLimit = 0;
    public int algaeCurrentLimit = 0;
  }

  public default void setAlgaeVoltage(double volts) {}
  ;

  public default void setCoralRPM(double rpm) {}
  ;

  public default void handSetEnableLimitSwitch(boolean enabled) {}
  ;

  public default boolean handIsCoralAtTarget() {
    return true;
  }

  public default void updateInputs(ArmInputs inputs) {}
  ;

  public default void setAlgaeRPM(double rpm) {}
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

  public default boolean isAlgaeAtTarget() {
    return true;
  }

  public default void handSetCoralCurrentLimit(int currentLimit) {}

  public default void handSetAlgaeCurrentLimit(int algaeCurrentLimit) {}
}
