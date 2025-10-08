package frc.robot.subsystems.arm;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.subsystems.constants.ArmConstants;

public class ArmIOSim implements ArmIO {

  private PIDController pid = ArmConstants.Gains.createPIDController();

  private ArmFeedforward feedforward = ArmConstants.Gains.createArmFF();
  private double targetAngleDeg = Units.radiansToDegrees(ArmConstants.kInitialAngleRad);

  private static final SingleJointedArmSim sim =
      new SingleJointedArmSim(
          DCMotor.getKrakenX60Foc(1),
          ArmConstants.kGearing,
          SingleJointedArmSim.estimateMOI(ArmConstants.kLength, ArmConstants.kMass),
          ArmConstants.kLength,
          Units.degreesToRadians(ArmConstants.kMinAngle),
          Units.degreesToRadians(ArmConstants.kMaxAngle),
          true,
          ArmConstants.kInitialAngleRad);

  public ArmIOSim() {
    // pid.enableContinuousInput(0, 2 * Math.PI);
  }

  public void setTargetAngle(double degrees) {
    this.targetAngleDeg = degrees;
  }

  @Override
  public void updateInputs(ArmInputs inputs) {
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
}
