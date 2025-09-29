package frc.robot.subsystems.intake;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.constants.IntakeConstants;
import frc.robot.util.LoggedTunableGains;

public class IntakeIOSim implements IntakeIO {

  private final DCMotor rollerMotor = DCMotor.getKrakenX60Foc(1);

  private PIDController IPPid = IntakeConstants.IPGains.createPIDController();
  private PIDController rollerPid = IntakeConstants.rollerGains.createPIDController();

  private static final SingleJointedArmSim IPSim = new SingleJointedArmSim(
      DCMotor.getKrakenX60Foc(1),
      IntakeConstants.kIPGearing,
      SingleJointedArmSim.estimateMOI(IntakeConstants.kIPLength, IntakeConstants.kIPMass),
      IntakeConstants.kIPLength,
      Units.degreesToRadians(IntakeConstants.kIPMinAngle),
      Units.degreesToRadians(IntakeConstants.kIPMaxAngle),
      true,
      IntakeConstants.kIPInitialAngleRad);

  public double setpoint = 0.0;

  public IntakeIOSim() {
    SmartDashboard.putBoolean("Intake has coral", false);
  }

  public void updateInputs(IntakeInputs inputs) {
  }

  // roller functions

  public void setRollerVoltage(double volts) {
  }

  public void enableLimitSwitch() {
  }

  // intake pivot functions
  public void setVoltage(double volts) {
  }

  public void setTargetAngle(double degrees) {
  }

  public void setPID(LoggedTunableGains pid) {
  }

  public void setServoPos(double pos) {
  }

  public void configureSoftLimits(double minDeg, double maxDeg) {
  }

  public boolean intakePivotIsAtTarget() {
    return true;
  }
}
