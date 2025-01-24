package frc.robot.subsystems.rollers;

import org.littletonrobotics.junction.AutoLog;

public interface RollersIO {

  @AutoLog
  public static class RollersInputs { // PascalCase
    public boolean tubeIsOn = false;
    public double tubeOutputVoltage = 0.0;
    public double tubeCurrentAmps = 0.0;
    public double tubeVelocityRPM = 0.0;

    public boolean algaeIsOn = false;
    public double algaeOutputVoltage = 0.0;
    public double algaeCurrentAmps = 0.0;
    public double algaeVelocityRPM = 0.0;

    public double commandedAlgaeRPM = 0.0;
    public double commandedTubeRPM = 0.0;
  }

  public default void updateInputs(RollersInputs inputs) {}
  ;

  public default void setTubeRPM(double rpm) {}
  ;

  public default void setAlgaeRPM(double rpm) {}
  ;
}
