package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  private NetworkTableInstance inst;
  private NetworkTable table;

  public Vision() {
    inst = NetworkTableInstance.getDefault();
    table = inst.getTable("slamdunk");
  }

  @Override
  public void periodic() {
    Logger.recordOutput(
        "Odometry/Vision",
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
                    table.getEntry("pose/rotation/q/z").getDouble(0.0)))));
  }
}
