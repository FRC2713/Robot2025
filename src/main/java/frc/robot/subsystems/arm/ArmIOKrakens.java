package frc.robot.subsystems.arm;

import au.grapplerobotics.LaserCan;
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
import frc.robot.subsystems.constants.IntakeConstants;
import frc.robot.util.LoggedTunableGains;
import frc.robot.util.PhoenixUtil;

public class ArmIOKrakens implements ArmIO {
  private LaserCan laserCan;

  private final TalonFX armMotor;
  private final TalonFX handMotor;
  private final CANcoder encoder;
  private double targetDegrees;
  private final MotionMagicExpoTorqueCurrentFOC angleRequest =
      new MotionMagicExpoTorqueCurrentFOC(0);
  private TalonFXConfiguration armMotorConfig;
  private CANcoderConfiguration encoderConfig;

  public ArmIOKrakens() {
    this.armMotor = new TalonFX(ArmConstants.ArmCANId);
    this.encoder = new CANcoder(ArmConstants.ArmEncoderCANId);
    this.handMotor = new TalonFX(ArmConstants.HandCANId);
    armMotorConfig = createArmKrakenConfig();
    encoderConfig = createCANcoderConfiguration();
    var handMotorConfig = createHandKrakenConfig();
    laserCan = new LaserCan(ArmConstants.kHandLaserCANId);
    PhoenixUtil.tryUntilOk(5, () -> this.encoder.getConfigurator().apply(encoderConfig, 0.25));
    PhoenixUtil.tryUntilOk(5, () -> armMotor.getConfigurator().apply(armMotorConfig, 0.25));
    PhoenixUtil.tryUntilOk(5, () -> handMotor.getConfigurator().apply(handMotorConfig, 0.25));
    PhoenixUtil.tryUntilOk(
        5, () -> armMotor.setPosition(encoder.getAbsolutePosition().getValueAsDouble(), 0.25));
  }

  public TalonFXConfiguration createHandKrakenConfig() {
    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Feedback.SensorToMechanismRatio = 1.0;
    // config.Feedback.RotorToSensorRatio = ArmConstants.kGearing;
    // config.TorqueCurrent.PeakForwardTorqueCurrent = ArmConstants.kStallCurrentLimit;
    // config.TorqueCurrent.PeakReverseTorqueCurrent = -ArmConstants.kStallCurrentLimit;
    // config.CurrentLimits.StatorCurrentLimit = ArmConstants.kStatorCurrentLimit;
    // config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.MotorOutput.Inverted =
        (ArmConstants.kHandInverted)
            ? InvertedValue.CounterClockwise_Positive
            : InvertedValue.Clockwise_Positive;

    config.ClosedLoopGeneral.ContinuousWrap = true;

    return config;
  }

  public TalonFXConfiguration createArmKrakenConfig() {
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
        (ArmConstants.kArmInverted)
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
  public void handSetVoltage(double volts) {
    handMotor.setVoltage(volts);
  }

  @Override
  public void setTargetAngle(double degrees) {
    this.targetDegrees = degrees;
    armMotor.setControl(angleRequest.withPosition(Units.degreesToRotations(degrees)));
  }

  @Override
  public void updateInputs(ArmInputs inputs) {
    inputs.velocityDPS = Units.rotationsToDegrees(armMotor.getVelocity().getValueAsDouble());
    inputs.armVoltage = armMotor.getMotorVoltage().getValueAsDouble();
    inputs.angleDegrees = Units.rotationsToDegrees(armMotor.getPosition().getValueAsDouble());
    inputs.absoluteAngleDegrees =
        Units.rotationsToDegrees(encoder.getAbsolutePosition().getValueAsDouble());
    inputs.commandedAngleDegs = targetDegrees;

    inputs.setpointVelocity = armMotor.getClosedLoopReference().getValueAsDouble();

    inputs.handVoltage = handMotor.getMotorVoltage().getValueAsDouble();
    inputs.hasCoral = hasCoral();
    inputs.laserCanDist = laserCan.getMeasurement().distance_mm;
  }

  @Override
  public void setPID(LoggedTunableGains pid) {
    armMotorConfig.Slot0 = pid.toTalonFX(GravityTypeValue.Arm_Cosine);
    armMotorConfig.MotionMagic = pid.getMotionMagicConfig();

    PhoenixUtil.tryUntilOk(5, () -> armMotor.getConfigurator().apply(armMotorConfig, 0.25));
  }

  @Override
  public void setBus(double bus) {
    armMotor.set(bus);
  }

  @Override
  public boolean isAtTarget() {
    return Math.abs(
            Units.rotationsToDegrees(armMotor.getPosition().getValueAsDouble())
                - this.targetDegrees)
        < ArmConstants.AT_TARGET_GIVE_DEGS;
  }

  public boolean hasCoral() {
    return laserCan.getMeasurement().distance_mm <= IntakeConstants.kLaserDistance;
  }
}
