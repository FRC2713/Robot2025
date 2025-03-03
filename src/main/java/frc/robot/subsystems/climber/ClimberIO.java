package frc.robot.subsystems.climber;

import frc.robot.util.LoggedTunableGains;
import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {

  @AutoLog
  public static class ClimberInputs {
    public double velocityDPS = 0.0;
    public double voltage = 0.0;
    public double angleDegrees = 0.0;
    public double amps = 0.0;

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
