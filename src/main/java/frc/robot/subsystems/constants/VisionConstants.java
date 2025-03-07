package frc.robot.subsystems.constants;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.util.LimelightInfo;

public class VisionConstants {
  // public static double CAMERA_TO_TAG_MAX_DIST_INCHES = 120;
  public static double MAX_POSE_JUMP_METERS = 0.5; // Units.feetToMeters(5);
  public static double MAX_SPEED = 0.02;

  public record PoseEstimatorErrorStDevs(double translationalStDev, double rotationalStDev) {
    public PoseEstimatorErrorStDevs multiplyByRange(double range) {
      return new PoseEstimatorErrorStDevs(this.translationalStDev * range, rotationalStDev);
    }

    public Matrix<N3, N1> toMatrix() {
      return VecBuilder.fill(
          this.translationalStDev, this.translationalStDev, this.rotationalStDev);
    }
  }

  public static PoseEstimatorErrorStDevs POSE_ESTIMATOR_STATE_STDEVS =
      new PoseEstimatorErrorStDevs(1.0, Units.degreesToRadians(1));
  public static PoseEstimatorErrorStDevs POSE_ESTIMATOR_MAX_SPEED_STDEVS =
      new PoseEstimatorErrorStDevs(2.0, Units.degreesToRadians(10));
  public static PoseEstimatorErrorStDevs POSE_ESTIMATOR_VISION_MULTI_TAG_STDEVS =
      new PoseEstimatorErrorStDevs(0.001, Units.degreesToRadians(10));
  public static PoseEstimatorErrorStDevs POSE_ESTIMATOR_VISION_DISABLED =
      new PoseEstimatorErrorStDevs(0.001, Units.degreesToRadians(1));

  public static final double MAX_TIME_DIFFERENCE = 0.5;

  // its probably bad OOD to have this but ¯\_(ツ)_/¯
  public static final VisionOptions ACTIVE_VISION_OPTION = VisionOptions.SLAMDUNK;

  public static final String VISION_SERVER_URL =
      Constants.currentMode == Mode.REAL
          ? "http://rhr-jetson-1:5801/"
          : "http://10.10.197.83:5173/";

  // TODO determine limelight locations
  public static LimelightInfo FRONT_LIMELIGHT_INFO =
      LimelightInfo.builder()
          .ntTableName("limelight-a")
          .location(
              new Transform3d(
                  Units.inchesToMeters(-10.072374),
                  Units.inchesToMeters(9.304687),
                  Units.inchesToMeters(24.149765),
                  new Rotation3d(
                      0,
                      Units.degreesToRadians(-14.209976),
                      Units.degreesToRadians(180 - 71.409653 + 90))))
          .mountingDirection(LimelightInfo.MountingDirection.HORIZONTAL_LL3)
          .build();
  public static LimelightInfo BACK_LIMELIGHT_INFO =
      LimelightInfo.builder()
          .ntTableName("limelight-b")
          .location(
              new Transform3d(
                  Units.inchesToMeters(-10.062783),
                  Units.inchesToMeters(-9.304855),
                  Units.inchesToMeters(24.185351),
                  new Rotation3d(
                      0,
                      Units.degreesToRadians(-14.209976),
                      Units.degreesToRadians(180 - 23.725656))))
          .mountingDirection(LimelightInfo.MountingDirection.HORIZONTAL_LL3)
          .build();

  public enum VisionOptions {
    WHEEL_ODOMETRY,
    SLAMDUNK,
    MEGATAG2
  }
}
