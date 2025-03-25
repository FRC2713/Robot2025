package frc.robot.subsystems.algaeClaw;

import org.littletonrobotics.junction.AutoLog;

public interface AlgaeClawIO {

  @AutoLog
  public static class AlgaeClawInputs { // PascalCase
    public double algaeClawOutputVoltage = 0.0;
    public double algaeClawCurrentAmps = 0.0;
    public double algaeClawVelocityRPM = 0.0;
    public double commandedAlgaeClawRPM = 0.0;
    public double algaeClawPositionDegs = 0.0;
    public boolean hasAlgae = false;
  }

  public default void updateInputs(AlgaeClawInputs inputs) {}
  ;

  public default void setRPM(double rpm) {}
  ;

  public default void setEnableLimitSwitch(boolean enabled) {}
  ;

  public default boolean isAtTarget() {
    return true;
  }
}
