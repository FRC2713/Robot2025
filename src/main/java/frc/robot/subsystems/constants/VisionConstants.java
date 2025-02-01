package frc.robot.subsystems.constants;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public class VisionConstants {
  public static double CAMERA_TO_TAG_MAX_DIST_INCHES = 120;
  public static double MAX_POSE_JUMP_METERS = 0.5; // Units.feetToMeters(5);

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
  public static PoseEstimatorErrorStDevs POSE_ESTIMATOR_VISION_SINGLE_TAG_STDEVS =
      new PoseEstimatorErrorStDevs(0.2, Units.degreesToRadians(99999));
  public static PoseEstimatorErrorStDevs POSE_ESTIMATOR_VISION_MULTI_TAG_STDEVS =
      new PoseEstimatorErrorStDevs(0.001, Units.degreesToRadians(99999));

  public static final boolean ENABLE_MEGATAG2 = true;
}
