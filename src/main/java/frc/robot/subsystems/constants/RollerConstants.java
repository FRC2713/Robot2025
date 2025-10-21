package frc.robot.subsystems.constants;

import frc.robot.util.ControlGains;

public class RollerConstants {
  public static final int kCANId = 4;

  public static final double kMOI = 0.001;
  public static final double kGearing = 17.0 / 35.0;

  public static final boolean kMotorInverted = false;

  // values from Rev velocity control examples
  public static final ControlGains PID = new ControlGains().p(1).d(0.1);

  public static final double kAcceptablePositionError = 1000; // rotations
  public static final double kMaxVelocity = 6000; // rpm
  public static final double kMaxAcceleration = 1000; // rpm / sec

  public static final double AT_TARGET_GIVE_RPM = 150;
}
