package frc.robot.subsystems.arm;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.constants.ArmConstants;
import frc.robot.util.LoggedTunableGains;
import frc.robot.util.PhoenixUtil;

public class ArmIOKrakens implements ArmIO {
  private final TalonFX motor;
  private final CANcoder encoder;
  private double targetDegrees;
  private final MotionMagicExpoTorqueCurrentFOC angleRequest =
      new MotionMagicExpoTorqueCurrentFOC(0);
  private TalonFXConfiguration motorConfig;
  private CANcoderConfiguration encoderConfig;

  public ArmIOKrakens() {
    this.motor = new TalonFX(ArmConstants.kCANId);
    this.encoder = new CANcoder(ArmConstants.kEncoderCANId);
    motorConfig = createKrakenConfig();
    encoderConfig = createCANcoderConfiguration();
    PhoenixUtil.tryUntilOk(5, () -> this.encoder.getConfigurator().apply(encoderConfig, 0.25));
    PhoenixUtil.tryUntilOk(5, () -> motor.getConfigurator().apply(motorConfig, 0.25));
    PhoenixUtil.tryUntilOk(
        5, () -> motor.setPosition(encoder.getAbsolutePosition().getValueAsDouble(), 0.25));
  }

  public TalonFXConfiguration createKrakenConfig() {
    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Feedback.FeedbackRemoteSensorID = this.encoder.getDeviceID();
    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    config.Feedback.SensorToMechanismRatio = 1.0;
    config.Feedback.RotorToSensorRatio = ArmConstants.kGearing;
    config.TorqueCurrent.PeakForwardTorqueCurrent = ArmConstants.kStallCurrentLimit;
    config.TorqueCurrent.PeakReverseTorqueCurrent = -ArmConstants.kStallCurrentLimit;
    config.CurrentLimits.StatorCurrentLimit = ArmConstants.kStatorCurrentLimit;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.MotorOutput.Inverted =
        (ArmConstants.kInverted)
            ? InvertedValue.CounterClockwise_Positive
            : InvertedValue.Clockwise_Positive;

    config.Slot0 = ArmConstants.Gains.toTalonFX(GravityTypeValue.Arm_Cosine);
    config.MotionMagic = ArmConstants.Gains.getMotionMagicConfig();
    config.ClosedLoopGeneral.ContinuousWrap = false;
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        Units.degreesToRotations(ArmConstants.kMaxAngle);
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        Units.degreesToRotations(ArmConstants.kMinAngle);

    return config;
  }

  public static CANcoderConfiguration createCANcoderConfiguration() {
    var config = new CANcoderConfiguration();
    config.MagnetSensor.MagnetOffset = ArmConstants.kAbsoluteEncoderOffset;
    config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1 / 3;

    return config;
  }

  @Override
  public void armSetVoltage(double volts) {
    motor.setVoltage(volts);
  }

  @Override
  public void setTargetAngle(double degrees) {
    this.targetDegrees = degrees;
    motor.setControl(angleRequest.withPosition(Units.degreesToRotations(degrees)));
  }

  @Override
  public void updateInputs(ArmInputs inputs) {
    inputs.velocityDPS = Units.rotationsToDegrees(motor.getVelocity().getValueAsDouble());
    inputs.armVoltage = motor.getMotorVoltage().getValueAsDouble();
    inputs.angleDegrees = Units.rotationsToDegrees(motor.getPosition().getValueAsDouble());
    inputs.absoluteAngleDegrees =
        Units.rotationsToDegrees(encoder.getAbsolutePosition().getValueAsDouble());
    inputs.commandedAngleDegs = targetDegrees;

    inputs.setpointVelocity = motor.getClosedLoopReference().getValueAsDouble();
  }

  @Override
  public void setPID(LoggedTunableGains pid) {
    motorConfig.Slot0 = pid.toTalonFX(GravityTypeValue.Arm_Cosine);
    motorConfig.MotionMagic = pid.getMotionMagicConfig();

    PhoenixUtil.tryUntilOk(5, () -> motor.getConfigurator().apply(motorConfig, 0.25));
  }

  @Override
  public void setBus(double bus) {
    motor.set(bus);
  }

  @Override
  public boolean isAtTarget() {
    return Math.abs(
            Units.rotationsToDegrees(motor.getPosition().getValueAsDouble()) - this.targetDegrees)
        < ArmConstants.AT_TARGET_GIVE_DEGS;
  }
}
