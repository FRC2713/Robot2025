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

    // public double algaeOutputVoltage = 0.0;
    // public double algaeCurrentAmps = 0.0;
    // public double algaeVelocityRPM = 0.0;
    // public double commandedAlgaeRPM = 0.0;

    public boolean hasCoral = false;
    public boolean hasAlgae = false;
  }

  public default void updateInputs(RollersInputs inputs) {}
  ;

  public default void setTubeRPM(double rpm) {}
  ;

  public default void setEnableLimitSwitch(boolean enabled) {}
  ;

  public default void setEnableAlgaeLimitSwitch(boolean enabled) {}
  ;
}
