package frc.robot.subsystems.vision;

import frc.robot.Constants.FieldConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.constants.VisionConstants;
import frc.robot.util.RHRUtil;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;

public class VisionIOPoseEstimator implements VisionIO {
  @Setter @Getter private boolean allowJumps = true;

  @Override
  public void update(VisionOutputs outputs) {

    var pose2d = outputs.pose().toPose2d();

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

    var speed = RHRUtil.speed(RobotContainer.driveSubsystem.getChassisSpeeds().toTwist2d(0.02));
    if (pose2d.getTranslation().getX() != 0.0 || pose2d.getTranslation().getY() != 0.0) {
      Logger.recordOutput("Vision/Adding Measurement", true);
      Logger.recordOutput("Vision/Speed", speed);
      if (speed > VisionConstants.MAX_SPEED) {
        Logger.recordOutput("Vision/Reasoning", "Moving more than max speed");
        RobotContainer.driveSubsystem.addVisionMeasurement(
            pose2d,
            outputs.timestamp(),
            VisionConstants.POSE_ESTIMATOR_VISION_SINGLE_TAG_STDEVS.toMatrix());
        return;
      }
      Logger.recordOutput("Vision/Reasoning", "All good!");
      RobotContainer.driveSubsystem.addVisionMeasurement(
          pose2d,
          outputs.timestamp(),
          VisionConstants.POSE_ESTIMATOR_VISION_MULTI_TAG_STDEVS.toMatrix());
      return;
    }

    Logger.recordOutput("Vision/Adding Measurement", false);
    Logger.recordOutput("Vision/Reasoning", "No pose data");
  }
}
