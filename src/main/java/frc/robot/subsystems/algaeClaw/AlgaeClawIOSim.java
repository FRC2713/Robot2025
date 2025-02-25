package frc.robot.subsystems.algaeClaw;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.constants.AlgaeClawConstants;

/** For the first implementation, the robot controls AlgaeClaw and Algae with a single NEO */
public class AlgaeClawIOSim implements AlgaeClawIO {

  private final DCMotor motor = DCMotor.getNEO(1);
  private final DCMotorSim sim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              motor, AlgaeClawConstants.kMOI, AlgaeClawConstants.kGearing),
          motor);
  private double commandedRPM;

  private boolean hasAlgae = false;

  public void updateInputs(AlgaeClawInputs inputs) {
    sim.setAngularVelocity(Units.rotationsPerMinuteToRadiansPerSecond(commandedRPM));
    sim.update(0.020);

    inputs.algaeClawCurrentAmps = sim.getCurrentDrawAmps();
    inputs.algaeClawOutputVoltage = sim.getInputVoltage();
    inputs.algaeClawVelocityRPM = sim.getAngularVelocityRPM();
    inputs.algaeClawPositionDegs = Units.radiansToDegrees(sim.getAngularPositionRad());

    inputs.commandedAlgaeClawRPM = commandedRPM;
    inputs.hasAlgae = hasAlgae;
  }

  @Override
  public void setRPM(double rpm) {
    this.commandedRPM = rpm;
  }
}
