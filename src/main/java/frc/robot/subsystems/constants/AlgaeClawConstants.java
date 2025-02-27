package frc.robot.subsystems.constants;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import frc.robot.util.ControlGains;

public class AlgaeClawConstants {
  public static final int kCANId = 3;

  public static final double kMOI = 0.001;
  public static final double kGearing = 12.0 / 32.0;

  public static final boolean kMotorInverted = true;

  // values from Rev velocity control examples
  public static final ControlGains PID = new ControlGains().p(0.0003);

  public static final double kAcceptablePositionError = 10; // rotations
  public static final double kMaxVelocity = 6000; // rpm
  public static final double kMaxAcceleration = 6000; // rpm / sec

  public static SparkFlexConfig createConfig(int currentLimit) {
    SparkFlexConfig config = new SparkFlexConfig();

    config.inverted(kMotorInverted);
    config.encoder.positionConversionFactor(kGearing);
    config.idleMode(IdleMode.kBrake);

    config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    config.limitSwitch.reverseLimitSwitchEnabled(true).reverseLimitSwitchType(Type.kNormallyOpen);
    config.smartCurrentLimit(currentLimit);
    PID.applyPID(config.closedLoop);

    // config.closedLoop.maxMotion.maxVelocity(kAlgaeMaxVelocity);
    // config.closedLoop.maxMotion.maxAcceleration(kAlgaeMaxAcceleration);

    return config;
  }

  public static final double AT_TARGET_GIVE_RPM = 150;
}
