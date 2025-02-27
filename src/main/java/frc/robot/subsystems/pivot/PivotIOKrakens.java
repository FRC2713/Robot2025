package frc.robot.subsystems.pivot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.constants.PivotConstants;
import frc.robot.util.LoggedTunablePID;
import frc.robot.util.PhoenixUtil;

public class PivotIOKrakens implements PivotIO {
  private final TalonFX motor;
  private final CANcoder encoder;
  private double targetDegrees;
  private final MotionMagicExpoTorqueCurrentFOC angleRequest =
      new MotionMagicExpoTorqueCurrentFOC(0);
  private TalonFXConfiguration motorConfig;
  private CANcoderConfiguration encoderConfig;

  public PivotIOKrakens() {
    this.motor = new TalonFX(PivotConstants.kCANId);
    this.encoder = new CANcoder(PivotConstants.kEncoderCANId);
    motorConfig = PivotConstants.createKrakenConfig();
    encoderConfig = PivotConstants.createCaNcoderConfiguration();
    PhoenixUtil.tryUntilOk(5, () -> encoder.getConfigurator().apply(encoderConfig, 0.25));
    PhoenixUtil.tryUntilOk(5, () -> motor.getConfigurator().apply(motorConfig, 0.25));
    PhoenixUtil.tryUntilOk(
        5, () -> motor.setPosition(encoder.getAbsolutePosition().getValueAsDouble(), 0.25));
  }

  @Override
  public void setVoltage(double volts) {
    motor.setVoltage(volts);
  }

  @Override
  public void setTargetAngle(double degrees) {
    this.targetDegrees = degrees;
    motor.setControl(
        angleRequest.withPosition(
            Units.degreesToRotations(degrees - PivotConstants.humanOffsetDegs)));
  }

  @Override
  public void updateInputs(PivotInputs inputs) {
    inputs.velocityDPS = Units.rotationsToDegrees(motor.getVelocity().getValueAsDouble());
    inputs.voltage = motor.getMotorVoltage().getValueAsDouble();
    inputs.angleDegrees =
        Units.rotationsToDegrees(motor.getPosition().getValueAsDouble())
            + PivotConstants.humanOffsetDegs;
    inputs.commandedAngleDegs = targetDegrees;
    inputs.setpointVelocity = motor.getClosedLoopReference().getValueAsDouble();
    inputs.absoluteAngleDegrees =
        Units.rotationsToDegrees(encoder.getAbsolutePosition().getValueAsDouble())
            + PivotConstants.humanOffsetDegs;
  }

  @Override
  public void setPID(LoggedTunablePID pid) {
    motorConfig.Slot0 = pid.toTalonFX(GravityTypeValue.Arm_Cosine);
    motorConfig.MotionMagic = pid.toMotionMagic();

    PhoenixUtil.tryUntilOk(5, () -> motor.getConfigurator().apply(motorConfig, 0.25));
  }

  @Override
  public void setBus(double bus) {
    motor.set(bus);
  }
}
