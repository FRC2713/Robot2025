package frc.robot.subsystems.intake;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.constants.IntakeConstants;
import frc.robot.util.LoggedTunableGains;

public class IntakeIOSim implements IntakeIO {

  private static final DCMotor rollerMotor = DCMotor.getKrakenX60Foc(1);

  private PIDController IPPid = IntakeConstants.IPGains.createPIDController();
  private PIDController rollerPid = IntakeConstants.rollerGains.createPIDController();

  private static final SingleJointedArmSim IPSim =
      new SingleJointedArmSim(
          DCMotor.getKrakenX60Foc(1),
          IntakeConstants.kIPGearing,
          SingleJointedArmSim.estimateMOI(IntakeConstants.kIPLength, IntakeConstants.kIPMass),
          IntakeConstants.kIPLength,
          Units.degreesToRadians(IntakeConstants.kIPMinAngle),
          Units.degreesToRadians(IntakeConstants.kIPMaxAngle),
          true,
          IntakeConstants.kIPInitialAngleRad);

  private static final DCMotorSim rollerSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              rollerMotor, IntakeConstants.kRollerMOI, IntakeConstants.kRollerGearing),
          rollerMotor);

  private ArmFeedforward IPFeedForward = IntakeConstants.IPGains.createArmFF();
  private double IPTargetAngleDeg = IntakeConstants.kIPInitialAngleDeg;

  private double commandedRollerVolts;

  public IntakeIOSim() {
    SmartDashboard.putBoolean("Intake has coral", false);
  }

  public void updateInputs(IntakeInputs inputs) {
    // Intake pivot updates
    double pidOutput =
        IPPid.calculate(IPSim.getAngleRads(), Units.degreesToRadians(IPTargetAngleDeg));
    double feedforwardOutput =
        IPFeedForward.calculate(IPSim.getAngleRads(), IPSim.getVelocityRadPerSec());
    double output = DriverStation.isEnabled() ? pidOutput + feedforwardOutput : 0;

    IPSim.setInputVoltage(output);
    IPSim.update(0.02);

    inputs.intakePivotAngleDegrees = Units.radiansToDegrees(IPSim.getAngleRads());
    inputs.intakePivotVelocityDPS = Units.radiansToDegrees(IPSim.getVelocityRadPerSec());
    inputs.intakePivotVoltage = output;

    inputs.commandedAngleDegs = IPTargetAngleDeg;

    // Roller updates
    rollerSim.setInputVoltage(commandedRollerVolts);
    rollerSim.update(0.02);

    inputs.rollerCurrentAmps = rollerSim.getCurrentDrawAmps();
    inputs.rollerVelocityRPM = rollerSim.getAngularVelocityRPM();
    inputs.commandedRollerRPM = commandedRollerVolts;
    inputs.rollerPositionDegs = Units.radiansToDegrees(rollerSim.getAngularPositionRad());
    inputs.rollerCurrentLimit = 0;
  }

  // roller functions

  public void setRollerVoltage(double volts) {
    this.commandedRollerVolts = volts;
  }

  public void enableLimitSwitch() {}

  // intake pivot functions
  public void setVoltage(double volts) {
    IPSim.setInputVoltage(volts);
  }

  public void setTargetAngle(double degrees) {
    this.IPTargetAngleDeg = degrees;
  }

  public void setPID(LoggedTunableGains pid) {
    this.IPPid = pid.createPIDController();
    IPFeedForward = pid.createArmFF();
  }

  public void setServoPos(double pos) {}

  public void configureSoftLimits(double minDeg, double maxDeg) {}

  public boolean intakePivotIsAtTarget() {
    return Math.abs(Units.radiansToDegrees(IPSim.getAngleRads()) - this.IPTargetAngleDeg)
        < IntakeConstants.kIPTargetGiveDegs;
  }
}
