package frc.robot.subsystems.rollers;

import org.littletonrobotics.junction.AutoLog;

public interface RollersIO {

  @AutoLog
  public static class RollersInputs { // PascalCase
    public double tubeOutputVoltage = 0.0;
    public double tubeCurrentAmps = 0.0;
    public double tubeVelocityRPM = 0.0;
    public double commandedTubeRPM = 0.0;
    public double tubePositionDegs = 0.0;

    public boolean hasCoral = false;
  }

  public default void updateInputs(RollersInputs inputs) {}
  ;

  public default void setRPM(double rpm) {}
  ;

  public default void setEnableLimitSwitch(boolean enabled) {}
  ;
}
