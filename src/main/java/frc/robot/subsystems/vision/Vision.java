package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
  private VisionIO io;

  // IMPORTANT: Vision must be initialized after the drive subsystem
  public Vision(VisionIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.update();
  }
}
