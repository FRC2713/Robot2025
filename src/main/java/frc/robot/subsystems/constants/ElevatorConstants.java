package frc.robot.subsystems.constants;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.util.ControlGains;

public class ElevatorConstants {
  // TODO: 100 & 101 are arbitrary and need to be changed.
  public static final int kLeftCANId = 61;
  public static final int kRightCANId = 62;

  public static final boolean kIsLeftInverted = false;
  public static final boolean kIsRightInverted = true;

  public static final int kStallCurrentLimit = 30; // amps, value from 2024
  public static final int kDisableCurrentLimit = 120; // amps, values from 2024

  public static final int kMaxCurrentLimit = 40;

  public static final double kGearReduction = 9.0;
  public static final double kCarriageMass = Units.lbsToKilograms(0.249377); // kg
  public static final double kDrumRadius = Units.inchesToMeters(1.0);
  public static final double kAcceptablePositionErrorInches = 2; // inches

  public static final double kRotationsToHeightConversion = (1 / 25.0 * Math.PI * kDrumRadius);

  public static final double kMinHeight = 0.0; // inches
  public static final double kMaxHeight = 52.5;
  public static final double kInitialHeight = 0.0;

  public static final ControlGains motionMagic =
      new ControlGains().p(0).a(0.0).v(0.0).s(0.0).mmCruiseVelo(0);
  public static final ControlGains TalonFF = new ControlGains().s(0.).v(0.).a(0.).g(0.001);

  public static final ControlGains PID = new ControlGains().p(0.0).trapezoidal(10, 10);
  public static final ControlGains FF = new ControlGains().g(0.001).v(0.0);

  public static final int mech2dWidth = 20;
  public static final Color8Bit mech2dColor = new Color8Bit(255, 255, 0);

  public static SparkMaxConfig createLeftSparkMaxConfig() {
    SparkMaxConfig config = new SparkMaxConfig();

    config.inverted(kIsLeftInverted);
    config.smartCurrentLimit(kStallCurrentLimit).secondaryCurrentLimit(kDisableCurrentLimit);

    config.encoder.positionConversionFactor(kRotationsToHeightConversion);

    PID.applyPID(config.closedLoop);

    return config;
  }

  public static SparkMaxConfig createRightSparkMaxConfig() {
    SparkMaxConfig config = new SparkMaxConfig();

    config.inverted(kIsRightInverted);
    config.smartCurrentLimit(kStallCurrentLimit).secondaryCurrentLimit(kDisableCurrentLimit);

    config.encoder.positionConversionFactor(kRotationsToHeightConversion);

    config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    PID.applyPID(config.closedLoop);

    return config;
  }

  public static TalonFXConfiguration createKrakenConfig(boolean inverted) {
    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Feedback.SensorToMechanismRatio = ElevatorConstants.kGearReduction;
    config.TorqueCurrent.PeakForwardTorqueCurrent = ElevatorConstants.kMaxCurrentLimit;
    config.TorqueCurrent.PeakReverseTorqueCurrent = -ElevatorConstants.kMaxCurrentLimit;
    // change this
    config.CurrentLimits.StatorCurrentLimit = ElevatorConstants.kMaxCurrentLimit;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.MotorOutput.Inverted =
        (inverted) ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive;

    var slot0Config = config.Slot0;
    slot0Config.kP = ElevatorConstants.PID.getKP();
    slot0Config.kI = ElevatorConstants.PID.getKI();
    slot0Config.kD = ElevatorConstants.PID.getKD();
    slot0Config.kG = ElevatorConstants.TalonFF.getKG();
    slot0Config.kS = ElevatorConstants.TalonFF.getKS();
    slot0Config.kV = ElevatorConstants.TalonFF.getKV();
    slot0Config.kA = ElevatorConstants.TalonFF.getKA();
    slot0Config.GravityType = GravityTypeValue.Elevator_Static;

    config.MotionMagic = motionMagic.createMMConfigs();
    return config;
  }
}
