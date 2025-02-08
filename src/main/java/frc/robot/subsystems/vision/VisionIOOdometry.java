package frc.robot.subsystems.vision;

import edu.wpi.first.networktables.GenericPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.RobotContainer;
import frc.robot.util.PoseAndTwist3d;
import org.littletonrobotics.junction.Logger;

public class VisionIOOdometry implements VisionIO {
  public NetworkTableInstance inst;
  public NetworkTable table;

  public GenericPublisher publisher;

  public PoseAndTwist3d pose;

  public VisionIOOdometry() {
    inst = NetworkTableInstance.getDefault();
    table = inst.getTable("slamdunk");
    publisher = table.getTopic("wheelbased").genericPublish("PoseAndTwist3d");
    pose =
        PoseAndTwist3d.from(
            RobotContainer.driveSubsystem.getPose(),
            RobotContainer.driveSubsystem.getChassisSpeeds());
  }

  @Override
  public void update() {
    pose.update(
        RobotContainer.driveSubsystem.getPose(), RobotContainer.driveSubsystem.getChassisSpeeds());
    Logger.recordOutput("wheelodometry", PoseAndTwist3d.struct, pose);
  }
}
