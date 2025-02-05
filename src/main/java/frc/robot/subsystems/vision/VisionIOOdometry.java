package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.RobotContainer;

public class VisionIOOdometry implements VisionIO {
  public NetworkTableInstance inst;
  public NetworkTable table;

  public DoublePublisher[] publishers;

  public VisionIOOdometry() {
    inst = NetworkTableInstance.getDefault();
    table = inst.getTable("slamdunk");
    publishers =
        new DoublePublisher[] {
          table.getDoubleTopic("wheels/translation/x").publish(),
          table.getDoubleTopic("wheels/translation/y").publish(),
          table.getDoubleTopic("wheels/translation/z").publish(),
          table.getDoubleTopic("wheels/rotation/q/x").publish(),
          table.getDoubleTopic("wheels/rotation/q/y").publish(),
          table.getDoubleTopic("wheels/rotation/q/z").publish(),
          table.getDoubleTopic("wheels/rotation/q/w").publish()
        };
  }

  @Override
  public void update(VisionOutputs outputs) {
    var wheelPose = RobotContainer.driveSubsystem.getWheelBasedPose();
    publishers[0].set(wheelPose.getTranslation().getX());
    publishers[1].set(wheelPose.getTranslation().getY());
    publishers[2].set(0.0);

    var quat = new Rotation3d(wheelPose.getRotation()).getQuaternion();

    publishers[3].set(quat.getX());
    publishers[4].set(quat.getY());
    publishers[5].set(quat.getZ());
    publishers[6].set(quat.getW());
  }
}
