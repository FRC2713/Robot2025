package frc.robot.subsystems.constants;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.util.ControlGains;
import frc.robot.util.LoggedTunableGains;
import frc.robot.util.RHRUtil;

public class ArmConstants {
  public static final int kCANId = 17;
  public static final int kEncoderCANId = 18;
  public static final boolean kInverted = false;

  public static final double kGearing = 36 * (48. / 16.);
  public static final double kLength = Units.inchesToMeters(18);
  public static final double kMass = 7.094328; // kg
  public static final double kHeight = Units.inchesToMeters(24);

  public static final double kMinAngle = (-345);
  public static final double kMaxAngle = (227);

  public static final double kInitialAngleRad = Units.degreesToRadians(-90);

  // transform from elevator origin to shoulder origin
  public static final Transform3d kInitialTransform =
      new Transform3d(
          0.0,
          Units.inchesToMeters(9.784055),
          Units.inchesToMeters(34.75),
          new Rotation3d(0, 0, 0));

  // pose in robot frame
  public static final Pose3d kInitialPose =
      ElevatorConstants.kInitialPose.transformBy(kInitialTransform);

  public static final int kStallCurrentLimit = 30; // amps
  public static final int kStatorCurrentLimit = 100; // also amps
  public static final double kMaxAngularVelocity = 5; // RPM

  public static final double kP = RHRUtil.modeDependentDouble(400., 10); // output/rotation
  public static final double kI = RHRUtil.modeDependentDouble(0, 0.06); // Integral of kP
  public static final double kD = RHRUtil.modeDependentDouble(40., 0); // output/error in velocity

  public static final double kG = RHRUtil.modeDependentDouble(6.5., 0.381);
  public static final double kV = RHRUtil.modeDependentDouble(0, 0); // kV * rev/s = volts
  public static final double kA = RHRUtil.modeDependentDouble(0, 0);
  public static final double kS = RHRUtil.modeDependentDouble(2.5., 0.); // Volts

  // Theorectical Max: ...
  public static final double kTrapezoidalMaxVelocity = 16.;
  public static final double kTrapezoidalMaxAcceleration = 5;
  public static final double kTrapezoidalMaxJerk = 0;
  public static final double kExponential_kV = 13;
  public static final double kExponential_kA = 0.8;

  public static final LoggedTunableGains Gains =
      new LoggedTunableGains(
          "Arm",
          new ControlGains()
              // PID
              .p(kP)
              .i(kI)
              .d(kD)
              // FF
              .g(kG)
              .s(kS)
              .v(kV)
              .a(kA)
              // Motion Magic
              .trapezoidal(
                  kTrapezoidalMaxVelocity, kTrapezoidalMaxAcceleration, kTrapezoidalMaxJerk)
              .expo_kV(kExponential_kV)
              .expo_kA(kExponential_kA));
  public static final LoggedTunableGains SlowGains = Gains.slowDown();

  public static final double kAbsoluteEncoderOffset = Units.degreesToRotations(274.57);

  public static final int mech2dWidth = 10;
  public static final Color8Bit mech2dColor = new Color8Bit(0, 255, 0);

  public static final double AT_TARGET_GIVE_DEGS = 3;
}
