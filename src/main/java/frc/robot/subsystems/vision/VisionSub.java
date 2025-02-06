package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.constants.VisionConstants;
import frc.robot.util.VisionOutputs;
import org.littletonrobotics.junction.Logger;

public class VisionSub extends SubsystemBase {
  private VisionIO io;
  private NetworkTableInstance inst;
  private NetworkTable table;
  private Pose3d pose;
  private Pose2d pose2d;
  private double lastTimestamp = -1;

  // IMPORTANT: Vision must be initialized after the drive subsystem
  public VisionSub(VisionIO io) {
    inst = NetworkTableInstance.getDefault();
    table = inst.getTable("slamdunk");
    RobotContainer.driveSubsystem.resetOdometry(new Pose2d());
    this.io = io;
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

    io.update(new VisionOutputs(time, pose));
  }
}
