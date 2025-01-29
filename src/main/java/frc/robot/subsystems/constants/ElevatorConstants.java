package frc.robot.subsystems.constants;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.util.ControlGains;

public class ElevatorConstants {
  // TODO: 100 is arbitrary and needs to be changed.
  public static final int kLeftCANId = 100;
  public static final int kRightCANId = 101;

  public static final boolean kIsLeftInverted = false;
  public static final boolean kIsRightInverted = true;

  public static final int kStallCurrentLimit = 30; // amps, value from 2024
  public static final int kDisableCurrentLimit = 120; // amps, values from 2024

  public static final double kRotationsToHeightConversion =
      (1 / 20.0 * Math.PI * 1.7567); // values from 2024

  public static final double kGearReduction = 9.0;
  public static final double kCarriageMass = Units.lbsToKilograms(0.249377); // kg
  public static final double kDrumRadius = Units.inchesToMeters(1.0);
  public static final double kAcceptablePositionErrorInches = 2; // inches

  public static final double kMinHeight = Units.inchesToMeters(0.0); // inches
  public static final double kMaxHeight = Units.inchesToMeters(52.5);
  public static final double kInitialHeight = Units.inchesToMeters(5);

  public static final ControlGains PID = new ControlGains().p(25.0).trapezoidal(10, 10);
  public static final ControlGains FF = new ControlGains().g(0.45).v(0.76);

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
}
