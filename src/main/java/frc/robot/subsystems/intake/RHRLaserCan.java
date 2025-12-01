package frc.robot.subsystems.intake;

import au.grapplerobotics.LaserCan;
import java.util.Optional;

public class RHRLaserCan extends LaserCan {
  public RHRLaserCan(int id) {
    super(id);
  }

  public Optional<Measurement> getMeasurementSafe() {
    Measurement measurement = super.getMeasurement();
    if (measurement == null) {
      return Optional.empty();
    }
    return Optional.of(measurement);
  }
}
