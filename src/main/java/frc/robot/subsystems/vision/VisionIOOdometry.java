package frc.robot.subsystems.vision;

import frc.robot.RobotContainer;
import frc.robot.util.PoseAndTwist3d;
import org.littletonrobotics.junction.Logger;

public class VisionIOOdometry implements VisionIO {
  public PoseAndTwist3d pose;

  public VisionIOOdometry() {
    pose =
        PoseAndTwist3d.from(
            RobotContainer.driveSubsystem.getPose(),
            RobotContainer.driveSubsystem.getChassisSpeeds());
  }

  @Override
  public void update() {
    pose.update(
        RobotContainer.driveSubsystem.getPose(), RobotContainer.driveSubsystem.getChassisSpeeds());
    Logger.recordOutput("slamdunk/wheelodometry", PoseAndTwist3d.struct, pose);
  }
}
