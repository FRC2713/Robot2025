package frc.robot.subsystems.constants;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.util.ControlGains;

public class PivotConstants {
  public static final int kCANId = 100;
  public static final boolean kInverted = true;

  public static final double kGearing = 48.0 / 18.0;
  public static final double kLength = Units.inchesToMeters(18);
  public static final double kMass = 7.094328; // kg
  public static final double kMinAngle = Units.degreesToRadians(0);
  public static final double kMaxAngle = Units.degreesToRadians(60);
  public static final double kInitialAngle = Units.degreesToRadians(30);
  public static final double kRotationConversionFactor = (1.0 / 225.0 * 10); // values from 2024

  public static final int kStallCurrentLimit = 30; // amps
  public static final int kStatorCurrentLimit = 100; // also amps
  public static final double kMaxAngularVelocity = 5; // RPM

  // TODO: might need diff controllers for up and down
  public static final ControlGains PID = new ControlGains().p(10).trapezoidal(3.0, 3.0);

  public static final ControlGains motionMagic =
      new ControlGains().p(42).a(0.01).v(0.01).s(0.01).mmCruiseVelo(0);
  public static final ControlGains TalonFF = new ControlGains().s(0.).v(0.).a(0.);

  public static final double kAbsoluteEncoderOffset = 118.7;

  public static final int mech2dWidth = 10;
  public static final Color8Bit mech2dColor = new Color8Bit(0, 255, 255);

  public static SparkMaxConfig createConfig() {
    SparkMaxConfig config = new SparkMaxConfig();

    config.inverted(kInverted);
    config.smartCurrentLimit(30);

    config.encoder.positionConversionFactor(kRotationConversionFactor);

    config.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
    config.closedLoop.maxMotion.maxVelocity(kMaxAngularVelocity);
    PID.applyPID(config.closedLoop);

    return config;
  }

  public static TalonFXConfiguration createKrakenConfig() {
    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Feedback.SensorToMechanismRatio = PivotConstants.kGearing;
    config.TorqueCurrent.PeakForwardTorqueCurrent = PivotConstants.kStallCurrentLimit;
    config.TorqueCurrent.PeakReverseTorqueCurrent = -PivotConstants.kStallCurrentLimit;
    config.CurrentLimits.StatorCurrentLimit = PivotConstants.kStatorCurrentLimit;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.MotorOutput.Inverted =
        (PivotConstants.kInverted)
            ? InvertedValue.CounterClockwise_Positive
            : InvertedValue.Clockwise_Positive;

    var slot0Config = config.Slot0;
    slot0Config.kP = PivotConstants.PID.getKP();
    slot0Config.kI = PivotConstants.PID.getKI();
    slot0Config.kD = PivotConstants.PID.getKD();
    slot0Config.kS = PivotConstants.TalonFF.getKS();
    slot0Config.kV = PivotConstants.TalonFF.getKV();
    slot0Config.kA = PivotConstants.TalonFF.getKA();

    config.MotionMagic = motionMagic.createMMConfigs();
    return config;
  }
}
