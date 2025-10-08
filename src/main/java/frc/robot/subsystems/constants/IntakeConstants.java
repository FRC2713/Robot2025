package frc.robot.subsystems.constants;

import edu.wpi.first.math.util.Units;
import frc.robot.util.ControlGains;
import frc.robot.util.LoggedTunableGains;
import frc.robot.util.RHRUtil;

public class IntakeConstants {
  public static final int kIntakePivotCANId = 41;
  public static final int kIntakePivotEncoderCANId = 42;

  public static final int kRollerCANId = 43;
  public static final int kRollerEncoderCANId = 44;

  // Intake pivot arm constants

  // TODO: use something less confusing than "IP"
  public static final double kIPGearing = 36 * (48. / 16.);

  public static final int kIPStallCurrentLimit = 30; // amps
  public static final int kIPStatorCurrentLimit = 100; // also amps

  public static final boolean kIPInverted = false;

  public static final double kIPMaxAngle = (90);
  public static final double kIPMinAngle = (-10);

  public static final double kIPLength = Units.inchesToMeters(11);
  public static final double kIPMass = Units.lbsToKilograms(10);

  public static final double kIPInitialAngleDeg = -10.0;
  public static final double kIPInitialAngleRad = Units.degreesToRadians(kIPInitialAngleDeg);

  public static final double IPkP = RHRUtil.modeDependentDouble(100., 100.); // output/rotation
  public static final double IPkI = 0.0; // Integral of kP
  public static final double IPkD =
      RHRUtil.modeDependentDouble(0., 0.); // output/error in velocity

  public static final double IPkG = RHRUtil.modeDependentDouble(0.074, 0.074);
  public static final double IPkV = RHRUtil.modeDependentDouble(0, 0); // kV * rev/s = voltvs
  public static final double IPkA = RHRUtil.modeDependentDouble(0., 0.);
  public static final double IPkS = RHRUtil.modeDependentDouble(0., 0.); // Volts

  public static final double kIPTrapezoidalMaxVelocity = 1.;
  public static final double kIPTrapezoidalMaxAcceleration = 4;
  public static final double kIPTrapezoidalMaxJerk = 10;
  public static final double kIPExponential_kV = 13;
  public static final double kIPExponential_kA = 0.8;

  public static final LoggedTunableGains IPGains =
      new LoggedTunableGains(
          "intake pivot",
          new ControlGains()
              // PID
              .p(IPkP)
              .i(IPkI)
              .d(IPkD)
              // FF
              .g(IPkG)
              .s(IPkS)
              .v(IPkV)
              .a(IPkA)
              // Motion Magic
              .trapezoidal(
                  kIPTrapezoidalMaxVelocity, kIPTrapezoidalMaxAcceleration, kIPTrapezoidalMaxJerk)
              .expo_kV(kIPExponential_kV)
              .expo_kA(kIPExponential_kA));
  public static final LoggedTunableGains IPSlowGains = IPGains.slowDown();

  public static final double kIPTargetGiveDegs = 3;

  // roller constants

  // TODO: set roller constants to actual roller constants
  public static final double kRollerGearing = 36 * (48. / 16.);

  public static final int kRollerStallCurrentLimit = 30; // amps
  public static final int kRollerStatorCurrentLimit = 100; // also amps

  public static final boolean kRollerInverted = false;

  public static final double kRollerMaxAngle = (90);
  public static final double kRollerMinAngle = (-10);

  public static final double RollerkP = RHRUtil.modeDependentDouble(600., 10); // output/rotation
  public static final double RollerkI = 0.0; // Integral of kP
  public static final double RollerkD =
      RHRUtil.modeDependentDouble(80., 0); // output/error in velocity

  public static final double RollerkG = RHRUtil.modeDependentDouble(8., 0.381);
  public static final double RollerkV = RHRUtil.modeDependentDouble(13, 0); // kV * rev/s = volts
  public static final double RollerkA = RHRUtil.modeDependentDouble(0.11, 0);
  public static final double RollerkS = RHRUtil.modeDependentDouble(1., 0.); // Volts

  public static final double kRollerTrapezoidalMaxVelocity = 1.;
  public static final double kRollerTrapezoidalMaxAcceleration = 4;
  public static final double kRollerTrapezoidalMaxJerk = 10;
  public static final double kRollerExponential_kV = 13;
  public static final double kRollerExponential_kA = 0.8;
  public static final double kRollerGrabSpeed = 42;
  public static double kRollerMOI = 0.001;
}
