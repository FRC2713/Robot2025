package frc.robot.subsystems.constants;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import frc.robot.util.ControlGains;
import frc.robot.util.LoggedTunableGains;

public class ClimberConstants {
  public static final int kLeftCANId = 5;
  public static final int kRightCANId = 6;
  public static final double kLength = 0.1;
  public static final double kInitialAngle = -90;

  public static final LoggedTunableGains Gains =
      new LoggedTunableGains(
          "Climber",
          new ControlGains()
              // PID
              .p(0)
              .i(0)
              .d(0)
              // FF
              .g(0)
              .s(0)
              .v(0)
              .a(0));

  public static final double AT_TARGET_GIVE_DEGS = 2;
  public static final boolean kLeftMotorInverted = false;
  public static final boolean kRightMotorInverted = true;
  public static final double kGearing = 1. / (25 * (60 / 40) * (36 / 12));
  public static final int currentLimitAmps = 200;
  public static final double kMinAngle = -95;
  public static final double kMaxAngle = 95;
  public static final double kMass = 1;

  public static SparkFlexConfig createLeftSparkConfig() {
    SparkFlexConfig config = new SparkFlexConfig();

    config.inverted(kLeftMotorInverted);
    config.encoder.positionConversionFactor(kGearing);
    config.idleMode(IdleMode.kBrake);

    config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    // config.limitSwitch.reverseLimitSwitchEnabled(true).reverseLimitSwitchType(Type.kNormallyOpen);
    config.smartCurrentLimit(currentLimitAmps);
    Gains.toControlGains().applyPID(config.closedLoop);

    return config;
  }

  public static SparkFlexConfig createRightSparkConfig() {
    SparkFlexConfig config = new SparkFlexConfig();

    config.inverted(kRightMotorInverted);
    config.encoder.positionConversionFactor(kGearing);
    config.idleMode(IdleMode.kBrake);

    config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    // config.limitSwitch.reverseLimitSwitchEnabled(true).reverseLimitSwitchType(Type.kNormallyOpen);
    config.smartCurrentLimit(currentLimitAmps);
    Gains.toControlGains().applyPID(config.closedLoop);

    return config;
  }
}
