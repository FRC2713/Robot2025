package frc.robot.subsystems.intake;

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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.constants.IntakeConstants;
import frc.robot.subsystems.constants.ShoulderConstants;
import frc.robot.util.LoggedTunableGains;
import frc.robot.util.PhoenixUtil;

public class IntakeIOKrakens implements IntakeIO {

  private final TalonFX intakeMotor;
  private final CANcoder intakeEncoder;
  private TalonFXConfiguration intakeMotorConfig;
  private CANcoderConfiguration intakeEncoderConfig;

  private final TalonFX rollerMotor;
  private final CANcoder rollerEncoder;
  private TalonFXConfiguration rollerMotorConfig;
  private CANcoderConfiguration rollerEncoderConfig;

  private double targetDegrees;

  private final MotionMagicExpoTorqueCurrentFOC angleRequest =
      new MotionMagicExpoTorqueCurrentFOC(0);

  public IntakeIOKrakens() {

    SmartDashboard.putBoolean("Intake has coral", false);

    // Intake pivot stuff
    this.intakeMotor = new TalonFX(IntakeConstants.kIntakePivotCANId);
    this.intakeEncoder = new CANcoder(IntakeConstants.kIntakePivotEncoderCANId);
    intakeMotorConfig = createIntakePivotKrakenConfig();
    intakeEncoderConfig = createCANcoderConfiguration();
    PhoenixUtil.tryUntilOk(
        5, () -> this.intakeEncoder.getConfigurator().apply(intakeEncoderConfig, 0.25));
    PhoenixUtil.tryUntilOk(5, () -> intakeMotor.getConfigurator().apply(intakeMotorConfig, 0.25));
    PhoenixUtil.tryUntilOk(
        5,
        () ->
            intakeMotor.setPosition(intakeEncoder.getAbsolutePosition().getValueAsDouble(), 0.25));

    // Roller stuff
    this.rollerMotor = new TalonFX(IntakeConstants.kRollerCANId);
    this.rollerEncoder = new CANcoder(IntakeConstants.kRollerEncoderCANId);
    rollerMotorConfig = createRollerKrakenConfig();
    rollerEncoderConfig = createCANcoderConfiguration();
    PhoenixUtil.tryUntilOk(
        5, () -> this.rollerEncoder.getConfigurator().apply(rollerEncoderConfig, 0.25));
    PhoenixUtil.tryUntilOk(5, () -> rollerMotor.getConfigurator().apply(rollerMotorConfig, 0.25));
    PhoenixUtil.tryUntilOk(
        5,
        () ->
            rollerMotor.setPosition(rollerEncoder.getAbsolutePosition().getValueAsDouble(), 0.25));
  }

  @Override
  public void updateInputs(IntakeInputs inputs) {

    // Intake pivot updates
    inputs.intakePivotVelocityDPS =
        Units.rotationsToDegrees(intakeMotor.getVelocity().getValueAsDouble());
    inputs.intakePivotVoltage = intakeMotor.getMotorVoltage().getValueAsDouble();
    inputs.intakePivotAngleDegrees =
        Units.rotationsToDegrees(intakeMotor.getPosition().getValueAsDouble());
    inputs.intakePivotAbsoluteAngleDegrees =
        Units.rotationsToDegrees(intakeEncoder.getAbsolutePosition().getValueAsDouble());
    inputs.commandedAngleDegs = targetDegrees;

    inputs.rollerVelocityRPM =
        Units.rotationsToDegrees(rollerMotor.getVelocity().getValueAsDouble());
    inputs.rollerOutputVoltage = rollerMotor.getMotorVoltage().getValueAsDouble();
    inputs.rollerPositionDegs =
        Units.rotationsToDegrees(rollerMotor.getPosition().getValueAsDouble());
    inputs.commandedRollerRPM = targetDegrees;

    // inputs.setpointVelocity = intakeMotor.getClosedLoopReference().getValueAsDouble();
  }

  // roller functions

  @Override
  public void setRollerVoltage(double volts) {
    rollerMotor.setVoltage(volts);
  }

  @Override
  public void enableLimitSwitch() {}

  // intake pivot functions
  @Override
  public void setVoltage(double volts) {
    intakeMotor.setVoltage(volts);
  }

  @Override
  public void setTargetAngle(double degrees) {
    this.targetDegrees = degrees;
    intakeMotor.setControl(angleRequest.withPosition(Units.degreesToRotations(degrees)));
  }

  @Override
  public void setPID(LoggedTunableGains pid) {
    intakeMotorConfig.Slot0 = pid.toTalonFX(GravityTypeValue.Arm_Cosine);
    intakeMotorConfig.MotionMagic = pid.getMotionMagicConfig();

    PhoenixUtil.tryUntilOk(5, () -> intakeMotor.getConfigurator().apply(intakeMotorConfig, 0.25));
  }

  @Override
  public boolean intakePivotIsAtTarget() {
    return true;
  }

  public TalonFXConfiguration createRollerKrakenConfig() {
    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Feedback.FeedbackRemoteSensorID = this.rollerEncoder.getDeviceID();
    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    config.Feedback.SensorToMechanismRatio = 1.0;
    config.Feedback.RotorToSensorRatio = IntakeConstants.kRollerGearing;
    config.TorqueCurrent.PeakForwardTorqueCurrent = IntakeConstants.kRollerStallCurrentLimit;
    config.TorqueCurrent.PeakReverseTorqueCurrent = -IntakeConstants.kRollerStallCurrentLimit;
    config.CurrentLimits.StatorCurrentLimit = IntakeConstants.kRollerStatorCurrentLimit;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.MotorOutput.Inverted =
        (IntakeConstants.kRollerInverted)
            ? InvertedValue.CounterClockwise_Positive
            : InvertedValue.Clockwise_Positive;

    config.Slot0 = IntakeConstants.rollerGains.toTalonFX(GravityTypeValue.Arm_Cosine);
    config.MotionMagic = IntakeConstants.rollerGains.getMotionMagicConfig();
    config.ClosedLoopGeneral.ContinuousWrap = false;
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        Units.degreesToRotations(IntakeConstants.kRollerMaxAngle);
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        Units.degreesToRotations(IntakeConstants.kRollerMinAngle);

    return config;
  }

  public TalonFXConfiguration createIntakePivotKrakenConfig() {
    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Feedback.FeedbackRemoteSensorID = this.intakeEncoder.getDeviceID();
    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    config.Feedback.SensorToMechanismRatio = 1.0;
    config.Feedback.RotorToSensorRatio = IntakeConstants.kIPGearing;
    config.TorqueCurrent.PeakForwardTorqueCurrent = IntakeConstants.kIPStallCurrentLimit;
    config.TorqueCurrent.PeakReverseTorqueCurrent = -IntakeConstants.kIPStallCurrentLimit;
    config.CurrentLimits.StatorCurrentLimit = IntakeConstants.kIPStatorCurrentLimit;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.MotorOutput.Inverted =
        (IntakeConstants.kIPInverted)
            ? InvertedValue.CounterClockwise_Positive
            : InvertedValue.Clockwise_Positive;

    config.Slot0 = IntakeConstants.IPGains.toTalonFX(GravityTypeValue.Arm_Cosine);
    config.MotionMagic = IntakeConstants.IPGains.getMotionMagicConfig();
    config.ClosedLoopGeneral.ContinuousWrap = false;
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        Units.degreesToRotations(IntakeConstants.kIPMaxAngle);
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        Units.degreesToRotations(IntakeConstants.kIPMinAngle);
    return config;
  }

  public static CANcoderConfiguration createCANcoderConfiguration() {
    var config = new CANcoderConfiguration();
    config.MagnetSensor.MagnetOffset = ShoulderConstants.kAbsoluteEncoderOffset;
    config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1 / 3;

    return config;
  }
}
