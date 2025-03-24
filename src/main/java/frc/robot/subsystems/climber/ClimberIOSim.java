package frc.robot.subsystems.climber;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.subsystems.constants.ClimberConstants;
import frc.robot.util.LoggedTunableGains;

public class ClimberIOSim implements ClimberIO {
  private static final SingleJointedArmSim sim =
      new SingleJointedArmSim(
          DCMotor.getNeoVortex(2),
          1 / ClimberConstants.kGearing,
          SingleJointedArmSim.estimateMOI(ClimberConstants.kLength, ClimberConstants.kMass),
          ClimberConstants.kLength,
          Units.degreesToRadians(ClimberConstants.kMinAngle),
          Units.degreesToRadians(ClimberConstants.kMaxAngle),
          true,
          Units.degreesToRadians(ClimberConstants.kInitialAngle));

  private PIDController pid = ClimberConstants.Gains.createPIDController();

  public ClimberIOSim() {
    pid.enableContinuousInput(0, 2 * Math.PI);
  }

  private ArmFeedforward feedforward = ClimberConstants.Gains.createArmFF();
  private double targetAngleDeg = ClimberConstants.kInitialAngle;
  private double volts = -1;

  private double servoPos;

  @Override
  public void updateInputs(ClimberInputs inputs) {
    double output = volts;

    sim.update(0.02);
    sim.setInputVoltage(output);

    inputs.leftAngleDegrees = Units.radiansToDegrees(sim.getAngleRads());
    inputs.leftVelocityDPS = Units.radiansToDegrees(sim.getVelocityRadPerSec());
    inputs.leftVoltage = output;
    inputs.leftAmps = sim.getCurrentDrawAmps();

    inputs.rightAngleDegrees = Units.radiansToDegrees(sim.getAngleRads());
    inputs.rightVelocityDPS = Units.radiansToDegrees(sim.getVelocityRadPerSec());
    inputs.rightVoltage = output;
    inputs.rightAmps = sim.getCurrentDrawAmps();

    inputs.servoCommandedPos = servoPos;

    inputs.commandedAngleDegs = targetAngleDeg;
  }

  // right now climber just uses voltage control
  @SuppressWarnings("unused")
  private void setVoltageWithPID() {
    var pos = Units.radiansToDegrees(sim.getAngleRads());
    var vel = sim.getVelocityRadPerSec();
    this.volts = this.pid.calculate( Units.radiansToDegrees(sim.getAngleRads()), this.targetAngleDeg) +  this.feedforward.calculate(pos, vel);
  }

  @Override
  public void setVoltage(double volts) {
    this.volts = volts;
  }

  @Override
  public void setTargetAngle(double degrees) {
    this.targetAngleDeg = degrees;
  }

  @Override
  public void setPID(LoggedTunableGains pid) {
    this.pid = pid.createPIDController();
    this.feedforward = pid.createArmFF();
  }

  @Override
  public void setServoPos(double pos) {
    servoPos = pos;
  }

  @Override
  public boolean isAtTarget() {
    return Math.abs(Units.radiansToDegrees(sim.getAngleRads()) - this.targetAngleDeg)
        < ClimberConstants.AT_TARGET_GIVE_DEGS;
  }
}
