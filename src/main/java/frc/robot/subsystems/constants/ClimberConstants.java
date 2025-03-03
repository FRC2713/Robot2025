package frc.robot.subsystems.constants;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import frc.robot.util.ControlGains;
import frc.robot.util.LoggedTunableGains;

public class ClimberConstants {
  public static final int kCANId = 5;
  public static final double kLength = 10;
  public static final double kInitialAngle = -90;

  public static final LoggedTunableGains PID =
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
  public static final boolean kMotorInverted = false;
  public static final double kGearing = 1;
  public static final int currentLimitAmps = 200;

  public static SparkFlexConfig createSparkConfig() {
    SparkFlexConfig config = new SparkFlexConfig();

    config.inverted(kMotorInverted);
    config.encoder.positionConversionFactor(kGearing);
    config.idleMode(IdleMode.kBrake);

    config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    config.limitSwitch.reverseLimitSwitchEnabled(true).reverseLimitSwitchType(Type.kNormallyOpen);
    config.smartCurrentLimit(currentLimitAmps);
    PID.toControlGains().applyPID(config.closedLoop);

    return config;
  }
}
