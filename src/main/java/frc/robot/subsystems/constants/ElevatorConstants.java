package frc.robot.subsystems.constants;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.util.ControlGains;
import frc.robot.util.LoggedTunableGains;
import frc.robot.util.RHRUtil;

public class ElevatorConstants {
  public static final int kLeftCANId = 61;
  public static final int kRightCANId = 62;

  public static final boolean kIsLeftInverted = false;
  public static final boolean kIsRightInverted = true;

  public static final int kStallCurrentLimit = 30; // amps, value from 2024
  public static final int kDisableCurrentLimit = 120; // amps, values from 2024

  public static final int kMaxCurrentLimit = 40;

  public static final double kGearReduction = 5.0;
  public static final double kCarriageMass = Units.lbsToKilograms(0.249377); // kg
  public static final double kDrumRadius = 1.0;
  public static final double kAcceptablePositionErrorInches = 2; // inches

  public static final double kRotationsToHeightConversion = (Math.PI * kDrumRadius * 2);

  public static final double kMinHeight = 0.0; // inches
  public static final double kMaxHeight = 26.0; // inches
  public static final double kInitialHeight = 0;

  public static double toInches(double rotations) {
    return rotations * kRotationsToHeightConversion;
  }

  public static double toRotations(double inches) {
    return inches / kRotationsToHeightConversion;
  }

  // robot frame to elevator frame
  public static final Transform3d kInitialTransform =
      new Transform3d(0.0, 0.0, Units.inchesToMeters(kInitialHeight), new Rotation3d(0, 0, 0));

  // pose in robot frame
  public static final Pose3d kInitialPose = new Pose3d().transformBy(kInitialTransform);

  public static final LoggedTunableGains Gains =
      new LoggedTunableGains(
          "Elevator_L_ONE",
          new ControlGains()
              // PID
              .p(RHRUtil.modeDependentDouble(25., 0.1))
              .i(0.0)
              .d(RHRUtil.modeDependentDouble(1, 0.01))
              // FF
              .g(RHRUtil.modeDependentDouble(12, 0.0785))
              .v(0.0)
              .a(0.0)
              // Motion Magic
              .trapezoidal(
                  RHRUtil.modeDependentDouble(0.5, 0.1),
                  RHRUtil.modeDependentDouble(0.5, 0.1),
                  RHRUtil.modeDependentDouble(0.5, 0.1))
              .exponential(6.4, 0.1));

  public static final double mech2dWidth = Units.inchesToMeters(1);
  public static final Color8Bit mech2dColor = new Color8Bit(255, 0, 0);

  public static final double AT_TARGET_GIVE_INCHES = 1.0;
}
