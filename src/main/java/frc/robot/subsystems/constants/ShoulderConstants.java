package frc.robot.subsystems.constants;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.util.ControlGains;
import frc.robot.util.LoggedTunableGains;
import frc.robot.util.RHRUtil;

public class ShoulderConstants {
  public static final int kCANId = 17;
  public static final int kEncoderCANId = 18;
  public static final boolean kInverted = false;

  public static final double kGearing = 180;
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
  public static final double kI = 0.0; // Integral of kP
  public static final double kD = RHRUtil.modeDependentDouble(80., 0); // output/error in velocity

  public static final double kG = RHRUtil.modeDependentDouble(3., 0.381);
  public static final double kV = RHRUtil.modeDependentDouble(22.34, 0); // kV * rev/s = volts
  public static final double kA = RHRUtil.modeDependentDouble(0.08, 0);
  public static final double kS = RHRUtil.modeDependentDouble(0., 0.); // Volts

  // Theorectical Max: .55555
  public static final double kTrapezoidalMaxVelocity = .5;
  public static final double kTrapezoidalMaxAcceleration = 1;
  public static final double kTrapezoidalMaxJerk = 10;
  public static final double kExponential_kV = 22.34;
  public static final double kExponential_kA = 0.8;

  public static final LoggedTunableGains Gains =
      new LoggedTunableGains(
          "Shoulder",
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

  public static final double kAbsoluteEncoderOffset = Units.degreesToRotations(163.125);

  public static final int mech2dWidth = 10;
  public static final Color8Bit mech2dColor = new Color8Bit(0, 255, 0);

  public static final double AT_TARGET_GIVE_DEGS = 3;
}
