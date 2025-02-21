package frc.robot.subsystems.algaeClaw;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.constants.RollerConstants;

/** For the first implementation, the robot controls Tube and Algae with a single NEO */
public class AlgaeClawIOSim implements AlgaeClawIO {

  private final DCMotor motor = DCMotor.getNEO(1);
  private final DCMotorSim sim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              motor, RollerConstants.kAlgaeMOI, RollerConstants.kAlgaeGearing),
          motor);
  private double commandedRPM;

  private boolean hasCoral = false;

  public void updateInputs(AlgaeClawInputs inputs) {
    sim.setAngularVelocity(Units.rotationsPerMinuteToRadiansPerSecond(commandedRPM));
    sim.update(0.020);

    inputs.tubeCurrentAmps = sim.getCurrentDrawAmps();
    inputs.tubeOutputVoltage = sim.getInputVoltage();
    inputs.tubeVelocityRPM = sim.getAngularVelocityRPM();
    inputs.tubePositionDegs = Units.radiansToDegrees(sim.getAngularPositionRad());

    inputs.commandedTubeRPM = commandedRPM;
    inputs.hasCoral = hasCoral;
  }

  @Override
  public void setTubeRPM(double rpm) {
    this.commandedRPM = rpm;
  }
}
