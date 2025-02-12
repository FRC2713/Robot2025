package frc.robot.subsystems.pivot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.constants.PivotConstants;
import frc.robot.util.LoggedTunablePID;
import frc.robot.util.PhoenixUtil;

public class PivotIOKrakens implements PivotIO {
  private final TalonFX motor;
  private double targetDegrees;
  private final MotionMagicExpoTorqueCurrentFOC angleRequest =
      new MotionMagicExpoTorqueCurrentFOC(0);
  private TalonFXConfiguration config;

  public PivotIOKrakens() {
    this.motor = new TalonFX(PivotConstants.kCANId);
    config = PivotConstants.createKrakenConfig();
    PhoenixUtil.tryUntilOk(5, () -> motor.getConfigurator().apply(config, 0.25));
    PhoenixUtil.tryUntilOk(
        5,
        () -> motor.setPosition(Units.radiansToRotations(PivotConstants.kInitialAngleRad), 0.25));
  }

  @Override
  public void setVoltage(double volts) {
    motor.setVoltage(volts);
  }

  @Override
  public void setTargetAngle(double degrees) {
    this.targetDegrees = degrees;
    motor.setControl(angleRequest.withPosition(Units.degreesToRotations(degrees)));
  }

  @Override
  public void updateInputs(PivotInputs inputs) {
    inputs.velocityDPS = Units.rotationsToDegrees(motor.getVelocity().getValueAsDouble());
    inputs.voltage = motor.getMotorVoltage().getValueAsDouble();
    inputs.angleDegrees = Units.rotationsToDegrees(motor.getPosition().getValueAsDouble());
    inputs.commandedAngleDegs = targetDegrees;
    inputs.setpointVelocity = motor.getClosedLoopReference().getValueAsDouble();
  }

  @Override
  public void setPID(LoggedTunablePID pid) {
    config.Slot0 = pid.toTalonFX(GravityTypeValue.Arm_Cosine);
    config.MotionMagic = pid.toMotionMagic();

    PhoenixUtil.tryUntilOk(5, () -> motor.getConfigurator().apply(config, 0.25));
  }

  @Override
  public void setBus(double bus) {
    motor.set(bus);
  }
}
