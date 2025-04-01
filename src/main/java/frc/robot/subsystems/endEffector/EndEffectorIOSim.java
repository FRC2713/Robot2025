package frc.robot.subsystems.endEffector;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.constants.RollerConstants;

/** For the first implementation, the robot controls Tube and Algae with a single NEO */
public class EndEffectorIOSim implements EndEffectorIO {

  public EndEffectorIOSim() {
    SmartDashboard.putBoolean("Has Coral", false);
    SmartDashboard.putBoolean("Has Algae", false);
  }

  private final DCMotor coralMotor = DCMotor.getNEO(1);
  private final DCMotorSim coralSim =
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

  public void updateInputs(EndEffectorInputs inputs) {
    coralSim.setAngularVelocity(Units.rotationsPerMinuteToRadiansPerSecond(commandedCoralRPM));
    coralSim.update(0.020);
    algaeSim.setAngularVelocity(Units.rotationsPerMinuteToRadiansPerSecond(commandedAlgaeRPM));
    algaeSim.update(0.020);

    inputs.tubeCurrentAmps = coralSim.getCurrentDrawAmps();
    inputs.tubeOutputVoltage = coralSim.getInputVoltage();
    inputs.tubeVelocityRPM = coralSim.getAngularVelocityRPM();
    inputs.tubePositionDegs = Units.radiansToDegrees(coralSim.getAngularPositionRad());

    inputs.algaeRollersCurrentAmps = algaeSim.getCurrentDrawAmps();
    inputs.algaeRollersOutputVoltage = algaeSim.getInputVoltage();
    inputs.algaeRollersVelocityRPM = algaeSim.getAngularVelocityRPM();
    inputs.algaeRollersPositionDegs = Units.radiansToDegrees(algaeSim.getAngularPositionRad());

    inputs.commandedTubeRPM = commandedCoralRPM;
    inputs.commandedAlgaeRollersRPM = commandedAlgaeRPM;
    inputs.hasCoral = SmartDashboard.getBoolean("Has Coral", false);
    inputs.hasAlgae = SmartDashboard.getBoolean("Has Algae", false);
  }

  @Override
  public void setCoralRPM(double rpm) {
    this.commandedCoralRPM = rpm;
  }

  @Override
  public void setAlgaeRPM(double rpm) {
    this.commandedAlgaeRPM = rpm;
  }

  @Override
  public boolean isCoralAtTarget() {
    return Math.abs(commandedCoralRPM - coralSim.getAngularVelocityRPM())
        < RollerConstants.AT_TARGET_GIVE_RPM;
  }

  @Override
  public boolean isAlgaeAtTarget() {
    return Math.abs(commandedAlgaeRPM - algaeSim.getAngularVelocityRPM())
        < RollerConstants.AT_TARGET_GIVE_RPM;
  }
}
