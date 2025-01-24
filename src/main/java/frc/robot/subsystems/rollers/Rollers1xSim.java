package frc.robot.subsystems.rollers;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.constants.RollerConstants;

/** For the first implementation, the robot controls Tube and Algae with a single NEO */
public class Rollers1xSim implements RollersIO {

  private final DCMotor algaeModel = DCMotor.getNEO(1);
  private final DCMotorSim algaeSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              algaeModel, RollerConstants.kAlgaeMOI, RollerConstants.kAlgaeGearing),
          algaeModel);
  private double commandedRPM;

  // private PIDController m_controller = RollerConstants.ALGAEPID.createPIDController();

  public void updateInputs(RollersInputs inputs) {
    // double pidOutput = m_controller.calculate(algaeSim.getAngularVelocityRPM(),
    // this.commandedRPM);
    // algaeSim.setInputVoltage(Math.signum(pidOutput) * Math.min(12, Math.abs(pidOutput)));

    algaeSim.setAngularVelocity(Units.rotationsPerMinuteToRadiansPerSecond(commandedRPM));
    algaeSim.update(0.020);

    inputs.algaeCurrentAmps = algaeSim.getCurrentDrawAmps();
    inputs.algaeOutputVoltage = algaeSim.getInputVoltage();
    inputs.algaeVelocityRPM = algaeSim.getAngularVelocityRPM();
    inputs.algaeIsOn = Math.abs(algaeSim.getAngularVelocityRPM()) > 10;

    inputs.commandedAlgaeRPM = commandedRPM;
  }

  public void setTubeRPM(double rpm) {
    this.commandedRPM = -rpm;
    algaeSim.setAngularVelocity(Units.rotationsPerMinuteToRadiansPerSecond(commandedRPM));
  }

  public void setAlgaeRPM(double rpm) {
    this.commandedRPM = rpm;
  }

  public double getTubeRPM() {
    return -algaeSim.getAngularVelocityRPM();
  }

  public double getAlgaeRPM() {
    return algaeSim.getAngularVelocityRPM();
  }
}
