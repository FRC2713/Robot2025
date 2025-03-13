package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.FieldConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.constants.VisionConstants;
import frc.robot.util.RHRUtil;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;

public class VisionIOPoseEstimator implements VisionIO {
  private NetworkTableInstance inst;
  private NetworkTable table;
  private Pose2d pose2d = new Pose2d();
  @Setter @Getter private boolean allowJumps = true;
  private double lastTimestamp = -1;
  private final double[] defaultPose = {0, 0, 0, 0, 0, 0, 0};

  // IMPORTANT: Vision must be initialized after the drive subsystem
  public VisionIOPoseEstimator() {
    inst = NetworkTableInstance.getDefault();
    table = inst.getTable("slamdunk");
  }

  @Override
  public void update() {
    var speed = RHRUtil.speed(RobotContainer.driveSubsystem.getChassisSpeeds().toTwist2d(0.02));
    Logger.recordOutput("Vision/Speed", speed);

    var poseArray = table.getEntry("pose").getDoubleArray(defaultPose);
    var pose =
        new Pose3d(
            new Translation3d(poseArray[0], poseArray[1], poseArray[2]),
            new Rotation3d(new Quaternion(poseArray[6], poseArray[3], poseArray[4], poseArray[5])));

    pose =
        pose.transformBy(new Transform3d(new Translation3d(), new Rotation3d(0, 0, Math.PI / 2)));

    pose2d = pose.toPose2d();

    Logger.recordOutput("Odometry/Vision3d", pose);
    Logger.recordOutput("Odometry/Vision", pose2d);

    var rawTime = table.getEntry("timeLeave").getDouble(0.0);

    var time = rawTime / 1e6;
    Logger.recordOutput("Vision/timeLeaveSec", time);

    var timeDiff = (Logger.getTimestamp() / 1e6) - time;
    Logger.recordOutput("Vision/timeDiffSec", timeDiff);

    if (lastTimestamp == time) {
      Logger.recordOutput("Vision/Adding Measurement", false);
      Logger.recordOutput("Vision/Reasoning", "No new data");
      return;
    }
    lastTimestamp = time;

    if (timeDiff > VisionConstants.MAX_TIME_DIFFERENCE) {
      Logger.recordOutput("Vision/Adding Measurement", false);
      Logger.recordOutput("Vision/Reasoning", "Time difference too large");
      return;
    }

    if (Math.signum(timeDiff) == -1) {
      Logger.recordOutput("Vision/Adding Measurement", false);
      Logger.recordOutput("Vision/Reasoning", "Got pose from the future?");
      return;
    }

    // Jump protection
    if ((pose2d
                .getTranslation()
                .getDistance(RobotContainer.driveSubsystem.getPose().getTranslation())
            > VisionConstants.MAX_POSE_JUMP_METERS
        && !allowJumps)) {
      Logger.recordOutput("Vision/Adding Measurement", false);
      Logger.recordOutput("Vision/Reasoning", "Jump protection");
      return;
    }

    // If out of field
    if (pose2d.getTranslation().getX() > FieldConstants.fieldLength
        || pose2d.getTranslation().getY() > FieldConstants.fieldWidth) {
      Logger.recordOutput("Vision/Adding Measurement", false);
      Logger.recordOutput("Vision/Reasoning", "Out of field");
      return;
    }

    if (pose2d.getTranslation().getX() != 0.0 || pose2d.getTranslation().getY() != 0.0) {
      Logger.recordOutput("Vision/Adding Measurement", true);
      if (speed > VisionConstants.MAX_SPEED) {
        Logger.recordOutput("Vision/Reasoning", "Moving more than max speed");
        RobotContainer.driveSubsystem.addVisionMeasurement(
            pose2d, time, VisionConstants.POSE_ESTIMATOR_MAX_SPEED_STDEVS.toMatrix());
        return;
      }
      if (DriverStation.isDisabled()) {
        Logger.recordOutput("Vision/Reasoning", "Disabled!");
        RobotContainer.driveSubsystem.addVisionMeasurement(
            pose2d, time, VisionConstants.POSE_ESTIMATOR_VISION_DISABLED.toMatrix());
      }
      Logger.recordOutput("Vision/Reasoning", "All good!");
      RobotContainer.driveSubsystem.addVisionMeasurement(
          pose2d, time, VisionConstants.POSE_ESTIMATOR_VISION_MULTI_TAG_STDEVS.toMatrix());
      return;
    }

    Logger.recordOutput("Vision/Adding Measurement", false);
    Logger.recordOutput("Vision/Reasoning", "No pose data");
  }

  @Override
  public Pose2d getPose() {
    return this.pose2d.getTranslation().getX() != 0 ? this.pose2d : null;
  }
}
