package frc.robot.subsystems.constants;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.util.ControlGains;

public class RollerConstants {
  public static final int kCoralCANId = 25;
  public static final int kAlgaeCANId = 101;

  public static final double kAlgaeMOI = 0.001;
  public static final double kAlgaeGearing = 1 / (48.0 / 18.0);
  public static final double kCoralMOI = 0.001;
  public static final double kCoralGearing = 1;

  public static final boolean kCoralMotorInverted = true;
  public static final boolean kAlgaeMotorInverted = true;

  // values from Rev velocity control examples
  public static final ControlGains CORALPID = new ControlGains().p(1).d(0.1);
  public static final ControlGains ALGAEPID = new ControlGains().p(0.0003);

  public static final double kCoralAcceptablePositionError = 1000; // rotations
  public static final double kCoralMaxVelocity = 6000; // rpm
  public static final double kCoralMaxAcceleration = 1000; // rpm / sec

  public static final double kAlgaeAcceptablePositionError = 10; // rotations
  public static final double kAlgaeMaxVelocity = 6000; // rpm
  public static final double kAlgaeMaxAcceleration = 6000; // rpm / sec

  public static SparkMaxConfig createCoralConfig(boolean enableLS) {
    SparkMaxConfig config = new SparkMaxConfig();

    config.inverted(kAlgaeMotorInverted);
    config.encoder.positionConversionFactor(kAlgaeGearing);
    config.idleMode(IdleMode.kBrake);
    config
        .limitSwitch
        .forwardLimitSwitchEnabled(enableLS)
        .forwardLimitSwitchType(Type.kNormallyOpen);

    config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    ALGAEPID.applyPID(config.closedLoop);

    // config.closedLoop.maxMotion.maxVelocity(kAlgaeMaxVelocity);
    // config.closedLoop.maxMotion.maxAcceleration(kAlgaeMaxAcceleration);

    return config;
  }

  public static final double AT_TARGET_GIVE_RPM = 150;

  public static final double kAlgaeCurrentThreshold = 10; // amps
}
