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

  private final TalonFX motor;
  private final CANcoder encoder;
  private TalonFXConfiguration motorConfig;
  private CANcoderConfiguration encoderConfig;
  private double targetDegrees;

  private final MotionMagicExpoTorqueCurrentFOC angleRequest = new MotionMagicExpoTorqueCurrentFOC(0);

  public IntakeIOKrakens() {

    SmartDashboard.putBoolean("Intake has coral", false);

    //Intake pivot stuff
    this.motor = new TalonFX(IntakeConstants.kCANId);
    this.encoder = new CANcoder(IntakeConstants.kEncoderCANId);
    motorConfig = createKrakenConfig();
    encoderConfig = createCANcoderConfiguration();
    PhoenixUtil.tryUntilOk(5, () -> this.encoder.getConfigurator().apply(encoderConfig, 0.25));
    PhoenixUtil.tryUntilOk(5, () -> motor.getConfigurator().apply(motorConfig, 0.25));
    PhoenixUtil.tryUntilOk(
        5, () -> motor.setPosition(encoder.getAbsolutePosition().getValueAsDouble(), 0.25));
  }

  public void updateInputs(IntakeInputs inputs) {

    //Intake pivot updates
    inputs.intakePivotVelocityDPS = Units.rotationsToDegrees(motor.getVelocity().getValueAsDouble());
    inputs.intakePivotVoltage = motor.getMotorVoltage().getValueAsDouble();
    inputs.intakePivotAngleDegrees = Units.rotationsToDegrees(motor.getPosition().getValueAsDouble());
    inputs.intakePivotAbsoluteAngleDegrees =
        Units.rotationsToDegrees(encoder.getAbsolutePosition().getValueAsDouble());
    inputs.commandedAngleDegs = targetDegrees;

    //inputs.setpointVelocity = motor.getClosedLoopReference().getValueAsDouble();
  }

  // roller functions
  public void setRollerRPM(double rpm) {}

  public void setRollerVoltage(double volts) {}

  public void setRollerCurrentLimit(int currentLimit) {}

  public boolean rollerIsAtTarget() {
    return true;
  }

  // intake pivot functions
  public void setVoltage(double volts) {
    motor.setVoltage(volts);
  }

  public void setTargetAngle(double degrees) {
    this.targetDegrees = degrees;
    motor.setControl(angleRequest.withPosition(Units.degreesToRotations(degrees)));
  }

  public void setPID(LoggedTunableGains pid) {}

  public void setServoPos(double pos) {}

  public void configureSoftLimits(double minDeg, double maxDeg) {}

  public boolean intakePivotIsAtTarget() {
    return true;
  }

  public TalonFXConfiguration createKrakenConfig() {
    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Feedback.FeedbackRemoteSensorID = this.encoder.getDeviceID();
    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    config.Feedback.SensorToMechanismRatio = 1.0;
    config.Feedback.RotorToSensorRatio = IntakeConstants.kGearing;
    config.TorqueCurrent.PeakForwardTorqueCurrent = IntakeConstants.kStallCurrentLimit;
    config.TorqueCurrent.PeakReverseTorqueCurrent = -IntakeConstants.kStallCurrentLimit;
    config.CurrentLimits.StatorCurrentLimit = IntakeConstants.kStatorCurrentLimit;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.MotorOutput.Inverted =
        (IntakeConstants.kInverted)
            ? InvertedValue.CounterClockwise_Positive
            : InvertedValue.Clockwise_Positive;

    config.Slot0 = IntakeConstants.Gains.toTalonFX(GravityTypeValue.Arm_Cosine);
    config.MotionMagic = IntakeConstants.Gains.getMotionMagicConfig();
    config.ClosedLoopGeneral.ContinuousWrap = false;
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        Units.degreesToRotations(IntakeConstants.kMaxAngle);
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        Units.degreesToRotations(IntakeConstants.kMinAngle);

    return config;
  }

  public static CANcoderConfiguration createCANcoderConfiguration() {
    var config = new CANcoderConfiguration();
    config.MagnetSensor.MagnetOffset = ShoulderConstants.kAbsoluteEncoderOffset;
    config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1 / 3;

    return config;
  }
}
