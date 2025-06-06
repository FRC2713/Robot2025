package frc.robot.subsystems.pivot;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.subsystems.constants.PivotConstants;
import frc.robot.util.LoggedTunableGains;

public class PivotIOSim implements PivotIO {

  private static final SingleJointedArmSim sim =
      new SingleJointedArmSim(
          DCMotor.getKrakenX60Foc(1),
          PivotConstants.kGearing,
          SingleJointedArmSim.estimateMOI(PivotConstants.kLength, PivotConstants.kMass),
          PivotConstants.kLength,
          PivotConstants.kMinAngleRad,
          PivotConstants.kMaxAngleRad,
          true,
          PivotConstants.kInitialAngleRad);

  private PIDController pid = PivotConstants.Gains.createPIDController();

  public PivotIOSim() {
    pid.enableContinuousInput(0, 2 * Math.PI);
  }

  private ArmFeedforward feedforward = PivotConstants.Gains.createArmFF();
  private double targetAngleDeg = Units.radiansToDegrees(PivotConstants.kInitialAngleRad);

  @Override
  public void updateInputs(PivotInputs inputs) {
    double pidOutput = pid.calculate(sim.getAngleRads(), Units.degreesToRadians(targetAngleDeg));
    double feedforwardOutput =
        feedforward.calculate(sim.getAngleRads(), sim.getVelocityRadPerSec());
    double output = DriverStation.isEnabled() ? pidOutput + feedforwardOutput : 0;

    sim.setInputVoltage(output);
    sim.update(0.02);

    inputs.angleDegrees = Units.radiansToDegrees(sim.getAngleRads());
    inputs.absoluteAngleDegrees = Units.radiansToDegrees(sim.getAngleRads());
    inputs.velocityDPS = Units.radiansToDegrees(sim.getVelocityRadPerSec());
    inputs.voltage = output;
    inputs.commandedAngleDegs = targetAngleDeg;
  }

  @Override
  public void setVoltage(double volts) {
    sim.setInputVoltage(volts);
  }

  @Override
  public void setTargetAngle(double degrees) {
    this.targetAngleDeg = degrees;
  }

  @Override
  public void setPID(LoggedTunableGains pid) {
    this.pid = pid.createPIDController();
    feedforward = pid.createArmFF();
  }

  @Override
  public boolean isAtTarget() {
    return Math.abs(Units.radiansToDegrees(sim.getAngleRads()) - targetAngleDeg)
        < PivotConstants.AT_TARGET_GIVE_DEGS;
  }
}
