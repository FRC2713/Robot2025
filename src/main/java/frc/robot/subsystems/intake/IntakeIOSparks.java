package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.intake.IntakeIO.IntakeInputs;
import frc.robot.util.LoggedTunableGains;

public class IntakeIOSparks implements IntakeIO {
  public IntakeIOSparks() {
    SmartDashboard.putBoolean("Intake has coral", false);
  }
  
  public void updateInputs(IntakeInputs inputs) {}

  //roller functions
  public void setRollerRPM(double rpm) {}

  public void setRollerVoltage(double volts) {}

  public void setRollerCurrentLimit(int currentLimit) {}

  public boolean rollerIsAtTarget() {
    return true;
  }

  //intake pivot functions
  public void setVoltage(double volts) {}

  public void setTargetAngle(double degrees) {}

  public void setPID(LoggedTunableGains pid) {}

  public void setServoPos(double pos) {}

  public void configureSoftLimits(double minDeg, double maxDeg) {}

  public boolean intakePivotIsAtTarget() {
    return true;
  }
}
