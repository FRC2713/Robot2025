package frc.robot.subsystems.elevator;

import frc.robot.util.LoggedTunablePID;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;

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
    public double commandedHeightInches = 0.0;
  }

  @AutoLogOutput(key = "Elevator/isAtTarget")
  public default boolean isAtTarget() {
    return false;
  }
  ;

  public default void updateInputs(ElevatorInputs inputs) {}
  ;

  public default void setVoltage(double volts1, double volts2) {}
  ;

  public default void setTargetHeight(double heightInches) {}
  ;

  public default void setPID(LoggedTunablePID pid) {}
}
