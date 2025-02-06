package frc.robot.subsystems.vision;

import frc.robot.util.VisionOutputs;

public interface VisionIO {
  public default void update(VisionOutputs outputs) {}
}
