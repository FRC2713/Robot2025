package frc.robot.subsystems.endEffector;

import org.littletonrobotics.junction.AutoLog;

public interface EndEffectorIO {

  @AutoLog
  public static class EndEffectorInputs { // PascalCase
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
  }

  public default void updateInputs(EndEffectorInputs inputs) {}
  ;

  public default void setAlgaeRPM(double rpm) {}
  ;

  public default void setCoralRPM(double rpm) {}
  ;

  public default void setEnableLimitSwitch(boolean enabled) {}
  ;

  public default boolean isCoralAtTarget() {
    return true;
  }

  public default boolean isAlgaeAtTarget() {
    return true;
  }
}
