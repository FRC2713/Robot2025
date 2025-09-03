package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotContainer;
import frc.robot.subsystems.constants.VisionConstants.PoseEstimatorErrorStDevs;
import org.littletonrobotics.junction.Logger;

public class VisionIOSLAMDunk implements VisionIO {
  DoubleArraySubscriber sub;
  double lastT = -1;
  Pose2d pose2d = null;

  public VisionIOSLAMDunk() {
    var inst = NetworkTableInstance.getDefault();
    var table = inst.getTable("slamdunk");
    sub = table.getDoubleArrayTopic("pose_robot").subscribe(new double[0]);
  }

  @Override
  public void update() {
    Logger.recordOutput("SLAMDunk/tFGPA", Timer.getFPGATimestamp());
    Logger.recordOutput("SLAMDunk/othert", Timer.getTimestamp());

    var poseArray = sub.get();
    if (poseArray.length > 0) {
      double t = poseArray[0];

      Logger.recordOutput("SLAMDunk/t", t);

      if (lastT != t) {
        lastT = t;
        var pose =
            new Pose3d(
                new Translation3d(poseArray[1], poseArray[2], poseArray[3]),
                new Rotation3d(
                    new Quaternion(poseArray[4], poseArray[5], poseArray[6], poseArray[7])));

        pose =
            pose.transformBy(
                new Transform3d(new Translation3d(), new Rotation3d(0, 0, Math.PI / 2)));

        pose2d = pose.toPose2d();

        RobotContainer.driveSubsystem.addVisionMeasurement(
            pose2d,
            Timer.getFPGATimestamp(),
            new PoseEstimatorErrorStDevs(0.001, Units.degreesToRadians(10)).toMatrix());

        Logger.recordOutput("Odometry/Vision3d", pose);
        Logger.recordOutput("Odometry/Vision", pose2d);
        Logger.recordOutput("Odometry/Latency", (System.currentTimeMillis() / 1000) - t);
        Logger.recordOutput("Odometry/hasPose", true);

        return;
      }
    }
    pose2d = null;
    Logger.recordOutput("Odometry/hasPose", false);
  }

  @Override
  public Pose2d getPose() {
    return pose2d;
  }
}
