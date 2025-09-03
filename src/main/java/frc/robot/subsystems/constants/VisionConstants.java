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
  public static double MAX_POSE_JUMP_METERS = 3; // Units.feetToMeters(5);
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

  // its probably bad OOD to have this but ¯\_(ツ)_/¯
  public static final VisionOptions ACTIVE_VISION_OPTION = VisionOptions.MEGATAG;

  public static PoseEstimatorErrorStDevs POSE_ESTIMATOR_STATE_STDEVS =
      new PoseEstimatorErrorStDevs(1.0, Units.degreesToRadians(1));
  public static PoseEstimatorErrorStDevs POSE_ESTIMATOR_MAX_SPEED_STDEVS =
      ACTIVE_VISION_OPTION == VisionOptions.SLAMDUNK
          ? new PoseEstimatorErrorStDevs(1.0, Units.degreesToRadians(999999))
          : new PoseEstimatorErrorStDevs(4.0, Units.degreesToRadians(999999));
  public static PoseEstimatorErrorStDevs POSE_ESTIMATOR_VISION_MULTI_TAG_STDEVS =
      ACTIVE_VISION_OPTION == VisionOptions.SLAMDUNK
          ? new PoseEstimatorErrorStDevs(1, Units.degreesToRadians(999999))
          : new PoseEstimatorErrorStDevs(2, Units.degreesToRadians(999999));
  public static PoseEstimatorErrorStDevs POSE_ESTIMATOR_VISION_DISABLED =
      new PoseEstimatorErrorStDevs(0.001, Units.degreesToRadians(999999));

  public static final double MAX_TIME_DIFFERENCE = 1.0;

  public static final String VISION_SERVER_URL =
      Constants.currentMode == Mode.REAL
          ? "http://rhr-jetson-1:5801/"
          : "http://10.10.197.83:5173/";

  // https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-coordinate-systems#robot-space

  // aka scoring side
  public static LimelightInfo FRONT_LIMELIGHT_INFO =
      LimelightInfo.builder()
          .ntTableName("limelight-a")
          .location(
              new Transform3d(
                  Units.inchesToMeters(7.522),
                  Units.inchesToMeters(-11.190),
                  Units.inchesToMeters(15.070),
                  new Rotation3d(0, 0, Units.degreesToRadians(-30))))
          .mountingDirection(LimelightInfo.MountingDirection.HORIZONTAL_LL3)
          .build();

  // aka intake side
  public static LimelightInfo BACK_LIMELIGHT_INFO =
      LimelightInfo.builder()
          .ntTableName("limelight-b")
          .location(
              new Transform3d(
                  Units.inchesToMeters(-6.594),
                  Units.inchesToMeters(-11.650),
                  Units.inchesToMeters(36.255),
                  new Rotation3d(0, 0, Units.degreesToRadians(180))))
          .mountingDirection(LimelightInfo.MountingDirection.HORIZONTAL_LL3)
          .build();

  public enum VisionOptions {
    SLAMDUNK_WHEEL_ODOMETRY,
    SLAMDUNK,
    SLAMDUNK_MEGATAG2_MERGED,
    MEGATAG2,
    MEGATAG
  }
}
