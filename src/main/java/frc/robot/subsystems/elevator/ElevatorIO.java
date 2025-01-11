package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {

  @AutoLog
  public static class ElevatorInputs {
    public double outputVoltageLeft = 0.0;
    public double heightInchesLeft = 0.0;
    public double velocityInchesPerSecondLeft = 0.0;
    public double tempCelsiusLeft = 0.0;
    public double currentDrawAmpsLeft = 0.0;

    public double outputVoltageRight = 0.0;
    public double heightInchesRight = 0.0;
    public double velocityInchesPerSecondRight = 0.0;
    public double tempCelsiusRight = 0.0;
    public double currentDrawAmpsRight = 0.0;

    public double commandedHeight = 0.0;
  }

  public default void updateInputs(ElevatorInputs inputs) {}
  ;

  public default void setVoltage(double volts1, double volts2) {}
  ;

  public default void setTargetHeight(double heightInches) {}
  ;

  // public void reset();

  // public boolean shouldApplyFF();

  // public void setCurrentLimits();
}
