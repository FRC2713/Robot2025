package frc.robot.subsystems.intake;

import au.grapplerobotics.LaserCan;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.constants.IntakeConstants;
import frc.robot.util.LoggedTunableGains;
import frc.robot.util.PhoenixUtil;

public class IntakeIOKrakens implements IntakeIO {
  private LaserCan laserCan;

  private final TalonFX pivotMotor;
  // private final CANcoder pivotEncoder;
  private TalonFXConfiguration pivotMotorConfig;
  // private CANcoderConfiguration pivotEncoderConfig;

  private final TalonFX rollerMotor;
  private TalonFXConfiguration rollerMotorConfig;

  private double targetDegrees;

  private final MotionMagicExpoTorqueCurrentFOC angleRequest =
      new MotionMagicExpoTorqueCurrentFOC(0);

  public IntakeIOKrakens() {

    SmartDashboard.putBoolean("Intake has coral", false);

    // Intake pivot stuff
    this.pivotMotor = new TalonFX(IntakeConstants.kIntakePivotCANId);
    // this.pivotEncoder = new CANcoder(IntakeConstants.kIntakePivotEncoderCANId);
    pivotMotorConfig = createIntakePivotKrakenConfig();
    // pivotEncoderConfig = createCANcoderConfiguration();
    // PhoenixUtil.tryUntilOk(
    //     5, () -> this.pivotEncoder.getConfigurator().apply(pivotEncoderConfig, 0.25));
    PhoenixUtil.tryUntilOk(5, () -> pivotMotor.getConfigurator().apply(pivotMotorConfig, 0.25));
    PhoenixUtil.tryUntilOk(
        5,
        () ->
            pivotMotor.setPosition(
                Units.degreesToRotations(IntakeConstants.kIPInitialAngleDeg), 0.25));

    // Roller stuff
    this.rollerMotor = new TalonFX(IntakeConstants.kRollerCANId);
    rollerMotorConfig = createRollerKrakenConfig();
    PhoenixUtil.tryUntilOk(5, () -> rollerMotor.getConfigurator().apply(rollerMotorConfig, 0.25));
    PhoenixUtil.tryUntilOk(5, () -> rollerMotor.setPosition(0.0, 0.25));

    laserCan = new LaserCan(IntakeConstants.kIntakeLaserCANId);
  }

  @Override
  public void updateInputs(IntakeInputs inputs) {

    // Intake pivot updates
    inputs.intakePivotVelocityDPS =
        Units.rotationsToDegrees(pivotMotor.getVelocity().getValueAsDouble());
    inputs.intakePivotVoltage = pivotMotor.getMotorVoltage().getValueAsDouble();
    inputs.intakePivotAngleDegrees =
        Units.rotationsToDegrees(pivotMotor.getPosition().getValueAsDouble());
    // inputs.intakePivotAbsoluteAngleDegrees =
    //     Units.rotationsToDegrees(pivotEncoder.getAbsolutePosition().getValueAsDouble());
    inputs.commandedAngleDegs = targetDegrees;

    inputs.rollerVelocityRPM =
        Units.rotationsToDegrees(rollerMotor.getVelocity().getValueAsDouble());
    inputs.rollerOutputVoltage = rollerMotor.getMotorVoltage().getValueAsDouble();
    inputs.rollerPositionDegs =
        Units.rotationsToDegrees(rollerMotor.getPosition().getValueAsDouble());
    inputs.commandedRollerRPM = targetDegrees;

    inputs.hasCoral = hasCoral();
    inputs.laserCanDist = laserCan.getMeasurement().distance_mm;

    // inputs.setpointVelocity = pivotMotor.getClosedLoopReference().getValueAsDouble();
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
    pivotMotor.setVoltage(volts);
  }

  @Override
  public void setTargetAngle(double degrees) {
    this.targetDegrees = degrees;
    pivotMotor.setControl(angleRequest.withPosition(Units.degreesToRotations(degrees)));
  }

  @Override
  public void setPID(LoggedTunableGains pid) {
    pivotMotorConfig.Slot0 = pid.toTalonFX(GravityTypeValue.Arm_Cosine);
    pivotMotorConfig.MotionMagic = pid.getMotionMagicConfig();

    PhoenixUtil.tryUntilOk(5, () -> pivotMotor.getConfigurator().apply(pivotMotorConfig, 0.25));
  }

  @Override
  public boolean intakePivotIsAtTarget() {
    return Math.abs(
            Units.rotationsToDegrees(pivotMotor.getPosition().getValueAsDouble())
                - this.targetDegrees)
        < IntakeConstants.AT_TARGET_GIVE_DEGS;
  }

  public TalonFXConfiguration createRollerKrakenConfig() {
    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
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
    return config;
  }

  public TalonFXConfiguration createIntakePivotKrakenConfig() {
    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    // config.Feedback.FeedbackRemoteSensorID = this.pivotEncoder.getDeviceID();
    // config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    config.Feedback.SensorToMechanismRatio = IntakeConstants.kIPGearing;
    config.Feedback.RotorToSensorRatio = 1.0;
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

  public boolean hasCoral() {
    return laserCan.getMeasurement().distance_mm <= IntakeConstants.kLaserDistance;
  }
  // public static CANcoderConfiguration createCANcoderConfiguration() {
  //   var config = new CANcoderConfiguration();
  //   config.MagnetSensor.MagnetOffset = ShoulderConstants.kAbsoluteEncoderOffset;
  //   config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1 / 3;

  //   return config;
  // }
}
