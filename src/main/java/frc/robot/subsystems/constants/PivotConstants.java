package frc.robot.subsystems.constants;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.util.ControlGains;

public class PivotConstants {
  public static final int kCANId = 100;
  public static final boolean kInverted = true;

  public static final double kGearing = 200;
  public static final double kLength = Units.inchesToMeters(18);
  public static final double kMass = 1.0; // kg
  public static final double kMinAngle = Units.degreesToRadians(0);
  public static final double kMaxAngle = Units.degreesToRadians(60);
  public static final double kInitialAngle = Units.degreesToRadians(30);
  public static final double kRotationConversionFactor = (1.0 / 225.0 * 10); // values from 2024

  public static final int kStallCurrentLimit = 30; // amps
  public static final double kMaxAngularVelocity = 5; // RPM

  // TODO: might need diff controllers for up and down
  public static final ControlGains PID = new ControlGains().p(10).trapezoidal(3.0, 3.0);

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
}
