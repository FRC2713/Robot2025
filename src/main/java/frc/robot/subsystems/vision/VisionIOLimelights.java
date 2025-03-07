package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.Drivetrain;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightInfo;
import org.littletonrobotics.junction.Logger;

public class VisionIOLimelights implements VisionIO {
  // TODO: totally untested!!!

  private final LimelightInfo primaryInfo;
  private final LimelightInfo secondaryInfo;

  private Pose2d pose;
  private double lastTimestamp;

  private CombinedMegaTagState state;

  public VisionIOLimelights(LimelightInfo primary, LimelightInfo secondary, Drivetrain drivetrain) {
    LimelightHelpers.SetRobotOrientation(
        secondary.getNtTableName(),
        RobotContainer.driveSubsystem.getPose().getRotation().getDegrees(),
        0,
        0,
        0,
        0,
        0);
    LimelightHelpers.SetRobotOrientation(
        primary.getNtTableName(),
        RobotContainer.driveSubsystem.getPose().getRotation().getDegrees(),
        0,
        0,
        0,
        0,
        0);

    this.primaryInfo = primary;
    this.secondaryInfo = secondary;
    this.state = CombinedMegaTagState.INIT;
  }

  @Override
  public void update() {
    if (RobotContainer.driveSubsystem.getAngularVelocityRadPerSec() > Units.degreesToRadians(720)) {
      this.state = CombinedMegaTagState.REJECTED_DUE_TO_SPIN_BLUR;
    }

    LimelightHelpers.PoseEstimate primaryMT2 =
        LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(primaryInfo.getNtTableName());
    LimelightHelpers.PoseEstimate secondaryMT2 =
        LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(secondaryInfo.getNtTableName());

    if (primaryMT2.tagCount > 0) {
      this.state = CombinedMegaTagState.UPDATED_WITH_PRIMARY;
    } else if (secondaryMT2.tagCount > 0) {
      this.state = CombinedMegaTagState.UPDATED_WITH_SECONDARY;
    } else {
      this.state = CombinedMegaTagState.REJECTED_DUE_TO_NO_TAGS;
    }

    if (this.state == CombinedMegaTagState.UPDATED_WITH_PRIMARY) {
      this.pose = primaryMT2.pose;
      this.lastTimestamp = primaryMT2.timestampSeconds;
    } else if (this.state == CombinedMegaTagState.UPDATED_WITH_SECONDARY) {
      this.pose = secondaryMT2.pose;
      this.lastTimestamp = secondaryMT2.timestampSeconds;
    }

    Logger.recordOutput("Vision/limelight state", this.state);

    Logger.recordOutput("Vision/" + primaryInfo.getNtTableName() + "/pose", primaryMT2.pose);
    Logger.recordOutput(
        "Vision/" + primaryInfo.getNtTableName() + "/tag count", primaryMT2.tagCount);
    Logger.recordOutput(
        "Vision/" + primaryInfo.getNtTableName() + "/timestamp", primaryMT2.timestampSeconds);

    Logger.recordOutput("Vision/" + secondaryInfo.getNtTableName() + "/pose", secondaryMT2.pose);
    Logger.recordOutput(
        "Vision/" + secondaryInfo.getNtTableName() + "/tag count", secondaryMT2.tagCount);
    Logger.recordOutput(
        "Vision/" + secondaryInfo.getNtTableName() + "/timestamp", secondaryMT2.timestampSeconds);
  }

  @Override
  public Pose2d getPose() {
    return this.pose;
  }

  @Override
  public double getTimestamp() {
    return this.lastTimestamp;
  }

  private enum CombinedMegaTagState {
    INIT,
    REJECTED_DUE_TO_SPIN_BLUR,
    REJECTED_DUE_TO_NO_TAGS,
    UPDATED_WITH_PRIMARY,
    UPDATED_WITH_SECONDARY
  }
}
