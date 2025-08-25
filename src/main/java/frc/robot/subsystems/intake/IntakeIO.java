package frc.robot.subsystems.intake;

import frc.robot.util.LoggedTunableGains;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

  @AutoLog
  public static class IntakeInputs {
    // intake pivot vars
    public double intakePivotVelocityDPS = 0.0;
    public double intakePivotVoltage = 0.0;
    public double intakePivotAngleDegrees = 0.0;
    public double intakePivotAbsoluteAngleDegrees = 0.0;
    public double intakePivotAmps = 0.0;

    public double commandedAngleDegs = 0.0;
    public double servoCommandedPos = 0.0;

    // Roller vars
    public double rollerOutputVoltage = 0.0;
    public double rollerCurrentAmps = 0.0;
    public double rollerVelocityRPM = 0.0;
    public double commandedRollerRPM = 0.0;
    public double rollerPositionDegs = 0.0;
    public int rollerCurrentLimit = 0;

    // Coral detection
    public boolean hasCoral = false;
    public double sensorDistance = 0.0;
  }

  public default void updateInputs(IntakeInputs inputs) {}

  // roller functions
  public default void setRollerRPM(double rpm) {}

  public default void setRollerVoltage(double volts) {}

  public default void setRollerCurrentLimit(int currentLimit) {}

  public default boolean rollerIsAtTarget() {
    return true;
  }

  // intake pivot functions
  public default void setVoltage(double volts) {}

  public default void setTargetAngle(double degrees) {}

  public default void setPID(LoggedTunableGains pid) {}

  public default void setServoPos(double pos) {}

  public default void configureSoftLimits(double minDeg, double maxDeg) {}

  public default boolean intakePivotIsAtTarget() {
    return true;
  }
}
