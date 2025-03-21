package frc.robot.subsystems.constants;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.util.ControlGains;
import frc.robot.util.LoggedTunableGains;
import frc.robot.util.RHRUtil;

public class PivotConstants {
  public static final int kCANId = 15;
  public static final int kEncoderCANId = 16;
  public static final boolean kInverted = false;

  public static final double kGearing = (36.0 / 12.0) * 25;
  public static final double kLength = Units.inchesToMeters(18);
  public static final double kMass = 7.094328; // kg

  public static final double kMinAngleRad = Units.degreesToRadians(-200);
  // Change may need to be reverted based on actual data. Note, however, that the pivot does need to
  // be able to move up to SSConstants.Pivot.Algae_L2_DEG in order to pick up algae off the reef.
  public static final double kMaxAngleRad = Units.degreesToRadians(200);

  public static final double kInitialAngleRad = Units.degreesToRadians(35);

  // transform from shoulder origin to pivot origin
  public static final Transform3d kInitialTransform =
      new Transform3d(
          -ShoulderConstants.kLength,
          Units.inchesToMeters(-4.675),
          0,
          new Rotation3d(0, -kInitialAngleRad, 0));

  // pose in robot frame
  public static final Pose3d kInitialPose =
      new Pose3d().transformBy(ShoulderConstants.kInitialTransform).transformBy(kInitialTransform);

  public static final int kStallCurrentLimit = 30; // amps
  public static final int kStatorCurrentLimit = 100; // also amps
  public static final double kMaxAngularVelocity = 5; // RPM

  public static final double kP = RHRUtil.modeDependentDouble(700., 10); // output/rotation
  public static final double kI = 0.0; // Integral of kP
  public static final double kD = RHRUtil.modeDependentDouble(70, 0); // output/error in velocity

  public static final double kG = RHRUtil.modeDependentDouble(8.5, 0.381);
  public static final double kV = RHRUtil.modeDependentDouble(9.31, 0); // kV * rev/s = volts
  public static final double kA = 0.0;
  public static final double kS = RHRUtil.modeDependentDouble(0.1, 0.); // Volts

  // Theoretical Max: 1.333
  public static final double kTrapezoidalMaxVelocity = .6;
  public static final double kTrapezoidalMaxAcceleration = 60;
  public static final double kTrapezoidalMaxJerk = 300;
  public static final double kExponential_kV = 9.31;
  public static final double kExponential_kA = 0.1;

  public static final LoggedTunableGains Gains =
      new LoggedTunableGains(
          "Pivot",
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
              .maxTrapezoidalVelocity(kTrapezoidalMaxVelocity)
              .maxTrapezoidalAcceleration(kTrapezoidalMaxAcceleration)
              .maxTrapezoidalJerk(kTrapezoidalMaxJerk)
              .expo_kV(kExponential_kV)
              .expo_kA(kExponential_kA));

  public static final double kAbsoluteEncoderOffset = Units.degreesToRotations(290.1953125 - 35);
  public static final double humanOffsetDegs = -55;

  public static final int mech2dWidth = 10;
  public static final Color8Bit mech2dColor = new Color8Bit(0, 0, 255);

  public static final double AT_TARGET_GIVE_DEGS = 3;
}
