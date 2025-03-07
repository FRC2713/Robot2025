package frc.robot.util;

import edu.wpi.first.math.geometry.Transform3d;
import lombok.Builder;
import lombok.Getter;

@Builder
public class LimelightInfo {
  public enum MountingDirection {
    // TODO determine these
    VERTICAL_LL3(49.7, 63.3),
    HORIZONTAL_LL3(63.3, 49.7);

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
}
