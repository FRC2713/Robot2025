package frc.robot.subsystems.pivot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.constants.PivotConstants;
import frc.robot.util.LoggedTunableGains;
import frc.robot.util.PhoenixUtil;

public class PivotIOKrakens implements PivotIO {
  private final TalonFX motor;
  private final CANcoder encoder;
  private double targetDegrees;
  private final MotionMagicTorqueCurrentFOC angleRequest = new MotionMagicTorqueCurrentFOC(0);
  private TalonFXConfiguration motorConfig;
  private CANcoderConfiguration encoderConfig;

  public PivotIOKrakens() {
    this.motor = new TalonFX(PivotConstants.kCANId);
    this.encoder = new CANcoder(PivotConstants.kEncoderCANId);
    motorConfig = createKrakenConfig();
    encoderConfig = createCaNcoderConfiguration();
    PhoenixUtil.tryUntilOk(5, () -> encoder.getConfigurator().apply(encoderConfig, 0.25));
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
    config.Feedback.RotorToSensorRatio = PivotConstants.kGearing;
    config.TorqueCurrent.PeakForwardTorqueCurrent = PivotConstants.kStallCurrentLimit;
    config.TorqueCurrent.PeakReverseTorqueCurrent = -PivotConstants.kStallCurrentLimit;
    config.CurrentLimits.StatorCurrentLimit = PivotConstants.kStatorCurrentLimit;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.MotorOutput.Inverted =
        (PivotConstants.kInverted)
            ? InvertedValue.CounterClockwise_Positive
            : InvertedValue.Clockwise_Positive;

    config.Slot0 = PivotConstants.Gains.toTalonFX(GravityTypeValue.Arm_Cosine);
    config.MotionMagic = PivotConstants.Gains.getMotionMagicConfig();
    config.ClosedLoopGeneral.ContinuousWrap = false;
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = PivotConstants.kMaxAngleRad;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = PivotConstants.kMinAngleRad;
    return config;
  }

  public CANcoderConfiguration createCaNcoderConfiguration() {
    var config = new CANcoderConfiguration();
    config.MagnetSensor.MagnetOffset = PivotConstants.kAbsoluteEncoderOffset;
    config.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;

    return config;
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
    inputs.isAtTarget =
        Math.abs(inputs.angleDegrees - targetDegrees) < PivotConstants.AT_TARGET_GIVE_DEGS;
    ;
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
}
