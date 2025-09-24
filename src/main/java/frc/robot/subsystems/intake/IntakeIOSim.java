package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.LoggedTunableGains;

public class IntakeIOSim implements IntakeIO {
  public IntakeIOSim() {
    SmartDashboard.putBoolean("Intake has coral", false);
  }

  public void updateInputs(IntakeInputs inputs) {}

  // roller functions
  public void setRollerRPM(double rpm) {}

  public void setRollerVoltage(double volts) {}

  public void setRollerCurrentLimit(int currentLimit) {}

  // intake pivot functions
  public void setVoltage(double volts) {}

  public void setTargetAngle(double degrees) {}

  public void setPID(LoggedTunableGains pid) {}

  public void setServoPos(double pos) {}

  public boolean intakePivotIsAtTarget() {
    return true;
  }
}
