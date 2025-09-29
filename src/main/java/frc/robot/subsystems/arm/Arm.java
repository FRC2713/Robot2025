package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  private final ArmInputsAutoLogged inputs;
  private final ArmIO IO;

  public Arm(ArmIO IO) {
    this.inputs = new ArmInputsAutoLogged();
    IO.updateInputs(inputs);
    this.IO = IO;
  }
  
  public void periodic() {

    if (ArmConstants.Gains.hasChanged(hashCode())) {
      this.IO.setPID(ArmConstants.Gains);
    }

    IO.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);
    this.updateTransform();
  }

  public void setTargetAngle(double degrees) {
    this.IO.setTargetAngle(degrees);
  }

  @AutoLogOutput(key = "Arm/isAtTarget")
  public boolean isAtTarget() {
    return this.IO.isAtTarget();
  }

  public double getCurrentAngle() {
    return this.inputs.angleDegrees;
  }

  public double getAbsoluteAngle() {
    return this.inputs.absoluteAngleDegrees;
  }
  
  public void setBus(double bus) {
    Logger.recordOutput("bus", bus);
    IO.setBus(bus);
  }

  private void updateTransform() {
    this.transform =
        new Transform3d(
            0.0, 0.0, 0.0, new Rotation3d(0, Units.degreesToRadians(inputs.angleDegrees), 0));

    this.pose =
             ArmConstants.kInitialPose
            .transformBy(RobotContainer.elevator.transform)
            .transformBy(this.transform);
  }
}
