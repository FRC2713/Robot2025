package frc.robot.subsystems.vision;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
  private VisionIO io;

  // IMPORTANT: Vision must be initialized after the drive subsystem
  public Vision(VisionIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.update();
  }

  public Pose2d getPose() {
    return io.getPose();
  }

  public double getTimestamp() {
    return io.getTimestamp();
  }

  public void updatePoseEstimate(SwerveDrivePoseEstimator poseEstimator) {
    this.io.updatePoseEstimate(poseEstimator);
  }

  public void resetPose(Pose2d pose) {
    io.resetPose(pose);
  }
}
