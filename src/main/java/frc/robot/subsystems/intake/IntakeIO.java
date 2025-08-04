package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeInputs {
    public boolean hasCoral = false;
    public double rollerOutputVoltage = 0.0;
    public double rollerCurrentAmps = 0.0;
    public double rollerVelocityRPM = 0.0;
    public double commandedRollerRPM = 0.0;
    public double rollerPositionDegs = 0.0;
    public double sensorDistance = 0.0;
    public int rollerCurrentLimit = 0;
  }

  public default void updateInputs(IntakeInputs inputs) {};

  public default void setRollerRPM(double rpm) {};

  public default void setRollerVoltage(double volts) {};

  public default void setRollerCurrentLimit(int currentLimit) {};

  public default boolean isAtTarget() {
    return true;
  }
}
