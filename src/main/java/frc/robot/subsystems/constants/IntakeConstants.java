package frc.robot.subsystems.constants;

import frc.robot.util.ControlGains;
import frc.robot.util.LoggedTunableGains;
import frc.robot.util.RHRUtil;

public class IntakeConstants {
    public static final int kCANId = 41;
    public static final int kEncoderCANId = 42;


    //Intake pivot arm constants
    public static final double kIPGearing = 36 * (48. / 16.);

    public static final int kIPStallCurrentLimit = 30; // amps
    public static final int kIPStatorCurrentLimit = 100; // also amps

    public static final boolean kIPInverted = false;

    public static final double kIPMaxAngle = (90);
    public static final double kIPMinAngle = (-10);

    public static final double IPkP = RHRUtil.modeDependentDouble(600., 10); // output/rotation
    public static final double IPkI = 0.0; // Integral of kP
    public static final double IPkD = RHRUtil.modeDependentDouble(80., 0); // output/error in velocity

    public static final double IPkG = RHRUtil.modeDependentDouble(8., 0.381);
    public static final double IPkV = RHRUtil.modeDependentDouble(13, 0); // kV * rev/s = volts
    public static final double IPkA = RHRUtil.modeDependentDouble(0.11, 0);
    public static final double IPkS = RHRUtil.modeDependentDouble(1., 0.); // Volts

    public static final double kIPTrapezoidalMaxVelocity = 1.;
    public static final double kIPTrapezoidalMaxAcceleration = 4;
    public static final double kIPTrapezoidalMaxJerk = 10;
    public static final double kIPExponential_kV = 13;
    public static final double kIPExponential_kA = 0.8;

    public static final LoggedTunableGains IPGains =
      new LoggedTunableGains(
          "Shoulder",
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
}
