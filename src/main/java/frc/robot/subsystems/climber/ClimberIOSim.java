package frc.robot.subsystems.climber;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.subsystems.constants.ClimberConstants;
import frc.robot.util.LoggedTunableGains;

public class ClimberIOSim implements ClimberIO {
  private static final SingleJointedArmSim sim =
      new SingleJointedArmSim(
          DCMotor.getNeoVortex(2),
          ClimberConstants.kGearing,
          SingleJointedArmSim.estimateMOI(ClimberConstants.kLength, ClimberConstants.kMass),
          ClimberConstants.kLength,
          Units.degreesToRadians(0),
          Units.degreesToRadians(360),
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
    double output = 0;
    if (volts == -1) {
      double pidOutput = pid.calculate(sim.getAngleRads(), Units.degreesToRadians(targetAngleDeg));
      double feedforwardOutput =
          feedforward.calculate(sim.getAngleRads(), sim.getVelocityRadPerSec());
      output = DriverStation.isEnabled() ? pidOutput + feedforwardOutput : 0;
    } else {
      output = volts;
    }

    sim.setInputVoltage(output);
    sim.update(0.02);

    inputs.leftAngleDegrees = Units.radiansToDegrees(sim.getAngleRads());
    inputs.leftVelocityDPS = Units.radiansToDegrees(sim.getVelocityRadPerSec());
    inputs.leftVoltage = output;

    inputs.rightAngleDegrees = Units.radiansToDegrees(sim.getAngleRads());
    inputs.rightVelocityDPS = Units.radiansToDegrees(sim.getVelocityRadPerSec());
    inputs.rightVoltage = output;

    inputs.servoCommandedPos = servoPos;

    inputs.commandedAngleDegs = targetAngleDeg;
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
    feedforward = pid.createArmFF();
  }

  @Override
  public void setServoPos(double pos) {
    servoPos = pos;
  }
}
