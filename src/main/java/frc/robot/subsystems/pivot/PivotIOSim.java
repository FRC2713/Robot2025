package frc.robot.subsystems.pivot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;
import frc.robot.subsystems.constants.PivotConstants;

public class PivotIOSim implements PivotIO {

  private static final SingleJointedArmSim sim =
      new SingleJointedArmSim(
          DCMotor.getNEO(2),
          PivotConstants.kGearing,
          SingleJointedArmSim.estimateMOI(PivotConstants.kLength, PivotConstants.kMass),
          PivotConstants.kLength,
          PivotConstants.kMinAngle,
          PivotConstants.kMaxAngle,
          true,
          PivotConstants.kInitialAngle);

  private PIDController controller = PivotConstants.PID.createPIDController();
  private double targetAngleDeg;

  public void updateInputs(PivotInputs inputs) {
    double pidOutput =
        controller.calculate(sim.getAngleRads(), Units.degreesToRadians(targetAngleDeg));
    sim.setInputVoltage(pidOutput);
    sim.update(Constants.simulationRate);

    inputs.angleDegrees = Units.radiansToDegrees(sim.getAngleRads());
    inputs.velocityDPS = Units.radiansToDegrees(sim.getVelocityRadPerSec());
    inputs.voltage = sim.getOutput(0);
    inputs.isOn = Math.abs(inputs.voltage) > 0.001;

    inputs.commandedAngle = Units.radiansToDegrees(controller.getSetpoint());
  }

  public void setVoltage(double volts) {
    sim.setInputVoltage(volts);
  }

  public void setTargetAngle(double degrees) {
    this.targetAngleDeg = degrees;
  }
}
