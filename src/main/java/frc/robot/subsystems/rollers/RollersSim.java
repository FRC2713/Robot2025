package frc.robot.subsystems.rollers;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.constants.RollerConstants;

public class RollersSim implements RollersIO {

  private final DCMotor tubeModel = DCMotor.getNEO(1);
  private final DCMotorSim tubeSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              tubeModel, RollerConstants.kCoralMOI, RollerConstants.kCoralGearing),
          tubeModel);

  private final DCMotor algaeModel = DCMotor.getNEO(1);
  private final DCMotorSim algaeSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              algaeModel, RollerConstants.kAlgaeMOI, RollerConstants.kAlgaeGearing),
          algaeModel);

  public void updateInputs(RollersInputs inputs) {
    inputs.algaeCurrentAmps = algaeSim.getCurrentDrawAmps();
    inputs.algaeOutputVoltage = algaeSim.getInputVoltage();
    inputs.algaeVelocityRPM = algaeSim.getAngularVelocityRPM();
    inputs.algaeIsOn = Math.abs(algaeSim.getAngularVelocityRPM()) > 10;

    inputs.tubeCurrentAmps = tubeSim.getCurrentDrawAmps();
    inputs.tubeOutputVoltage = tubeSim.getInputVoltage();
    inputs.tubeVelocityRPM = tubeSim.getAngularVelocityRPM();
    inputs.tubeIsOn = Math.abs(tubeSim.getAngularVelocityRPM()) > 10;
  }

  public void setTubeRPM(double rpm) {
    tubeSim.setAngularVelocity(rpm * ((2 * Math.PI) / 60));
  }

  public void setAlgaeRPM(double rpm) {
    algaeSim.setAngularVelocity(rpm * ((2 * Math.PI) / 60));
  }
}
