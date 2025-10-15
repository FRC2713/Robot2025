package frc.robot.subsystems.arm;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.subsystems.constants.ArmConstants;
import frc.robot.subsystems.constants.RollerConstants;

public class ArmIOSim implements ArmIO {

  private PIDController pid = ArmConstants.Gains.createPIDController();

  private ArmFeedforward feedforward = ArmConstants.Gains.createArmFF();
  private double targetAngleDeg = Units.radiansToDegrees(ArmConstants.kInitialAngleRad);

  private static final SingleJointedArmSim armSim =
      new SingleJointedArmSim(
          DCMotor.getKrakenX60Foc(1),
          ArmConstants.kGearing,
          SingleJointedArmSim.estimateMOI(ArmConstants.kLength, ArmConstants.kMass),
          ArmConstants.kLength,
          Units.degreesToRadians(ArmConstants.kMinAngle),
          Units.degreesToRadians(ArmConstants.kMaxAngle),
          true,
          ArmConstants.kInitialAngleRad);

  private final DCMotor handMotor = DCMotor.getKrakenX60Foc(1);
  private final DCMotorSim handSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              handMotor, RollerConstants.kMOI, RollerConstants.kGearing),
          handMotor);

  public ArmIOSim() {
    // pid.enableContinuousInput(0, 2 * Math.PI);
  }

  public void setTargetAngle(double degrees) {
    this.targetAngleDeg = degrees;
  }

  @Override
  public void updateInputs(ArmInputs inputs) {
    double armPidOutput =
        pid.calculate(armSim.getAngleRads(), Units.degreesToRadians(targetAngleDeg));
    double feedforwardOutput =
        feedforward.calculate(armSim.getAngleRads(), armSim.getVelocityRadPerSec());
    double armOutput = DriverStation.isEnabled() ? armPidOutput + feedforwardOutput : 0;
    ;

    armSim.setInputVoltage(armOutput);
    armSim.update(0.02);

    handSim.update(0.02);
    handSim.setInputVoltage(inputs.handVoltage);
    ;

    inputs.handRPM = handSim.getAngularVelocityRPM();
    inputs.angleDegrees = Units.radiansToDegrees(armSim.getAngleRads());
    inputs.velocityDPS = Units.radiansToDegrees(armSim.getVelocityRadPerSec());
    inputs.armVoltage = armOutput;

    inputs.commandedAngleDegs = targetAngleDeg;
  }
}
