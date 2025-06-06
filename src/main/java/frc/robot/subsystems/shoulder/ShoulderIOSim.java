package frc.robot.subsystems.shoulder;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.subsystems.constants.ShoulderConstants;
import frc.robot.util.LoggedTunableGains;

public class ShoulderIOSim implements ShoulderIO {

  private static final SingleJointedArmSim sim =
      new SingleJointedArmSim(
          DCMotor.getKrakenX60Foc(1),
          ShoulderConstants.kGearing,
          SingleJointedArmSim.estimateMOI(ShoulderConstants.kLength, ShoulderConstants.kMass),
          ShoulderConstants.kLength,
          Units.degreesToRadians(ShoulderConstants.kMinAngle),
          Units.degreesToRadians(ShoulderConstants.kMaxAngle),
          true,
          ShoulderConstants.kInitialAngleRad);

  private PIDController pid = ShoulderConstants.Gains.createPIDController();

  public ShoulderIOSim() {
    // pid.enableContinuousInput(0, 2 * Math.PI);
  }

  private ArmFeedforward feedforward = ShoulderConstants.Gains.createArmFF();
  private double targetAngleDeg = Units.radiansToDegrees(ShoulderConstants.kInitialAngleRad);

  @Override
  public void updateInputs(ShoulderInputs inputs) {
    double pidOutput = pid.calculate(sim.getAngleRads(), Units.degreesToRadians(targetAngleDeg));
    double feedforwardOutput =
        feedforward.calculate(sim.getAngleRads(), sim.getVelocityRadPerSec());
    double output = DriverStation.isEnabled() ? pidOutput + feedforwardOutput : 0;

    sim.setInputVoltage(output);
    sim.update(0.02);

    inputs.angleDegrees = Units.radiansToDegrees(sim.getAngleRads());
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
    return Math.abs(Units.radiansToDegrees(sim.getAngleRads()) - this.targetAngleDeg)
        < ShoulderConstants.AT_TARGET_GIVE_DEGS;
  }
}
