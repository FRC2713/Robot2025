package frc.robot.subsystems.climber;

import frc.robot.util.LoggedTunableGains;
import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {

  @AutoLog
  public static class ClimberInputs {
    public double leftVelocityDPS = 0.0;
    public double rightVelocityDPS = 0.0;
    public double leftVoltage = 0.0;
    public double rightVoltage = 0.0;
    public double leftAngleDegrees = 0.0;
    public double rightAngleDegrees = 0.0;
    public double leftAmps = 0.0;
    public double rightAmps = 0.0;

    public double commandedAngleDegs = 0.0;
  }

  public default void updateInputs(ClimberInputs inputs) {}
  ;

  public default void setVoltage(double volts) {}
  ;

  public default void setTargetAngle(double degrees) {}
  ;

  public default void setPID(LoggedTunableGains pid) {}
  ;

  public default void setServoPos(double pos) {}
  ;
}
