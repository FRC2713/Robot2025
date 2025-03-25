package frc.robot.subsystems.endEffector;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.constants.RollerConstants;

/** For the first implementation, the robot controls Tube and Algae with a single NEO */
public class EndEffectorIOSim implements EndEffectorIO {

  private final DCMotor coralMotor = DCMotor.getNEO(1);
  private final DCMotorSim sim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              coralMotor, RollerConstants.kMOI, RollerConstants.kGearing),
          coralMotor);
  private final DCMotor algaeMotor = DCMotor.getNEO(1);
  private final DCMotorSim algaeSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              algaeMotor, RollerConstants.kMOI, RollerConstants.kGearing),
          algaeMotor);
  private double commandedCoralRPM;
  private double commandedAlgaeRPM;

  private boolean hasCoral = true;

  public void updateInputs(EndEffectorInputs inputs) {
    sim.setAngularVelocity(Units.rotationsPerMinuteToRadiansPerSecond(commandedCoralRPM));
    sim.update(0.020);

    inputs.tubeCurrentAmps = sim.getCurrentDrawAmps();
    inputs.tubeOutputVoltage = sim.getInputVoltage();
    inputs.tubeVelocityRPM = sim.getAngularVelocityRPM();
    inputs.tubePositionDegs = Units.radiansToDegrees(sim.getAngularPositionRad());

    inputs.commandedTubeRPM = commandedCoralRPM;
    inputs.commandedAlgaeRollersRPM = commandedAlgaeRPM;
    inputs.hasCoral = hasCoral;
  }

  @Override
  public void setCoralRPM(double rpm) {
    this.commandedCoralRPM = rpm;
  }

  @Override
  public boolean isCoralAtTarget() {
    return Math.abs(commandedCoralRPM - sim.getAngularVelocityRPM())
        < RollerConstants.AT_TARGET_GIVE_RPM;
  }

  @Override
  public boolean isAlgaeAtTarget() {
    return Math.abs(commandedAlgaeRPM - algaeSim.getAngularVelocityRPM())
        < RollerConstants.AT_TARGET_GIVE_RPM;
  }
}
