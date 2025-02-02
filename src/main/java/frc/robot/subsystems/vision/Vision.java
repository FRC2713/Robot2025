package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.constants.VisionConstants;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  private NetworkTableInstance inst;
  private NetworkTable table;
  private Pose3d pose;
  private Pose2d pose2d;
  @Setter @Getter private boolean allowJumps = false;

  // IMPORTANT: Vision must be initialized after the drive subsystem
  public Vision() {
    inst = NetworkTableInstance.getDefault();
    table = inst.getTable("slamdunk");
    RobotContainer.driveSubsystem.resetOdometry(new Pose2d());
  }

  @Override
  public void periodic() {
    pose =
        new Pose3d(
            new Translation3d(
                table.getEntry("pose/translation/x").getDouble(0.0),
                table.getEntry("pose/translation/y").getDouble(0.0),
                table.getEntry("pose/translation/z").getDouble(0.0)),
            new Rotation3d(
                new Quaternion(
                    table.getEntry("pose/rotation/q/w").getDouble(0.0),
                    table.getEntry("pose/rotation/q/x").getDouble(0.0),
                    table.getEntry("pose/rotation/q/y").getDouble(0.0),
                    table.getEntry("pose/rotation/q/z").getDouble(0.0))));

    pose2d = pose.toPose2d();

    Logger.recordOutput("Odometry/Vision3d", pose);
    Logger.recordOutput("Odometry/Vision", pose2d);

    var rawTime = table.getEntry("timeLeave").getDouble(0.0);

    var timeDiff = Logger.getTimestamp() - rawTime;
    Logger.recordOutput("Vision/timeDiff", timeDiff);
    var time = rawTime / 1e6;
    Logger.recordOutput("Vision/timeLeaveSec", time);

    if (timeDiff > (0.5 * 1e6)) {
      Logger.recordOutput("Vision/Adding Measurement", false);
      Logger.recordOutput("Vision/Reasoning", "Time difference too large");
      return;
    }

    // Jump protection
    if (pose2d
                .getTranslation()
                .getDistance(RobotContainer.driveSubsystem.getPose().getTranslation())
            > 2
        && !allowJumps) {
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
      Logger.recordOutput("Vision/Reasoning", "All good!");
      RobotContainer.driveSubsystem.addVisionMeasurement(
          pose2d, time, VisionConstants.POSE_ESTIMATOR_VISION_MULTI_TAG_STDEVS.toMatrix());
      return;
    }

    Logger.recordOutput("Vision/Adding Measurement", false);
    Logger.recordOutput("Vision/Reasoning", "No pose data");
  }
}
