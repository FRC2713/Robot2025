package frc.robot.subsystems.vision;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {

  @AutoLog
  public static class VisionInputs {
    public Pose2d pose;
    public double timestamp;
    public int tagCount;
    public double averageTagSize;
    public double averagTagDistance;

    public double translationStddev;
    public double rotationStddev;
    public boolean applyVisionToPoseEstimate;
  }

  public default void update() {}

  public default Pose2d getPose() {
    return new Pose2d();
  }

  public default void resetPose(Pose2d pose) {}
  ;

  public default double getTimestamp() {
    return 0;
  }

  public default void updatePoseEstimate(SwerveDrivePoseEstimator poseEstimator) {}
  ;
}
