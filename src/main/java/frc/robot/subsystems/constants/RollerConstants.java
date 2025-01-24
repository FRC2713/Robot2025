package frc.robot.subsystems.constants;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.util.ControlGains;

public class RollerConstants {
  public static final int kCoralCANId = 101; // unitless
  public static final int kAlgaeCANId = 102; // unitless

  public static final double kAlgaeMOI = 0.001;
  public static final double kAlgaeGearing = 1;
  public static final double kCoralMOI = 0.001;
  public static final double kCoralGearing = 1;

  // values from Rev velocity control examples
  public static final ControlGains CORALPID = new ControlGains().p(1).d(0.1); // unitless
  public static final ControlGains ALGAEPID = new ControlGains().p(1).d(0.1); // unitless

  public static final double kCoralAcceptablePositionError = 1000; // rotations
  public static final double kCoralMaxVelocity = 6000; // rpm
  public static final double kCoralMaxAcceleration = 1000; // rpm / sec

  public static final double kAlgaeAcceptablePositionError = 10; // rotations
  public static final double kAlgaeMaxVelocity = 6000; // rpm
  public static final double kAlgaeMaxAcceleration = 1000; // rpm / sec

  public static SparkMaxConfig createCoralConfig() {
    SparkMaxConfig config = new SparkMaxConfig();

    config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    config.closedLoop.maxMotion.maxVelocity(kCoralMaxVelocity);
    config.closedLoop.maxMotion.maxAcceleration(kCoralMaxAcceleration);
    config.closedLoop.maxMotion.allowedClosedLoopError(kCoralAcceptablePositionError);
    CORALPID.applyPID(config.closedLoop);

    return config;
  }

  public static SparkMaxConfig createAlgaeConfig() {
    SparkMaxConfig config = new SparkMaxConfig();

    config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    config.closedLoop.maxMotion.maxVelocity(kAlgaeMaxVelocity);
    config.closedLoop.maxMotion.maxAcceleration(kAlgaeMaxAcceleration);
    CORALPID.applyPID(config.closedLoop);

    return config;
  }
}
