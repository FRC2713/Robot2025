package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;

public interface VisionIO {
  public default void update() {}

  public default Pose2d getPose() {
    return new Pose2d();
  }

  public default void resetPose(Pose2d pose) {}
  ;
}
