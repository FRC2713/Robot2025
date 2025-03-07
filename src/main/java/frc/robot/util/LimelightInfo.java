package frc.robot.util;

import edu.wpi.first.math.geometry.Transform3d;
import lombok.Builder;
import lombok.Getter;

@Builder
public class LimelightInfo {
  public enum MountingDirection {
    VERTICAL_LL3(49.7, 63.3),
    HORIZONTAL_LL3(63.3, 49.7); // MegaTag2 only supports Horizontal

    @Getter private final double horizontalFOV;
    @Getter private final double verticalFOV;

    MountingDirection(double horizontalFOV, double verticalFOV) {
      this.horizontalFOV = horizontalFOV;
      this.verticalFOV = verticalFOV;
    }
  }

  @Getter private String ntTableName;
  @Getter private Transform3d location;
  @Getter private MountingDirection mountingDirection;

  public void setCameraPose_RobotSpace() {
    LimelightHelpers.setCameraPose_RobotSpace(
        this.ntTableName,
        this.location.getX(),
        this.location.getY(),
        this.location.getZ(),
        this.location.getRotation().getX(),
        this.location.getRotation().getY(),
        this.location.getRotation().getZ());
  }
}
