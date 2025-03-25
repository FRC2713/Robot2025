package frc.robot.subsystems.shoulder;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.constants.ShoulderConstants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Shoulder extends SubsystemBase {
  private final ShoulderInputsAutoLogged inputs;
  private final ShoulderIO IO;
  public final MechanismLigament2d stage0 =
      new MechanismLigament2d(
          "stage0", ShoulderConstants.kHeight, 0, 7, new Color8Bit(255, 255, 255));
  public final MechanismLigament2d mech2d =
      new MechanismLigament2d(
          "shoulder",
          ShoulderConstants.kLength,
          ShoulderConstants.kInitialAngleRad,
          ShoulderConstants.mech2dWidth,
          ShoulderConstants.mech2dColor);

  public Pose3d pose = ShoulderConstants.kInitialPose;
  public Transform3d transform = new Transform3d();

  public Shoulder(ShoulderIO IO) {
    this.inputs = new ShoulderInputsAutoLogged();
    IO.updateInputs(inputs);
    this.IO = IO;
  }

  public void periodic() {
    if (ShoulderConstants.Gains.hasChanged(hashCode())) {
      this.IO.setPID(ShoulderConstants.Gains);
    }

    IO.updateInputs(inputs);
    Logger.processInputs("Shoulder", inputs);
    this.updateTransform();
  }

  public void setTargetAngle(double degrees) {
    this.IO.setTargetAngle(degrees);
  }

  @AutoLogOutput(key = "Shoulder/isAtTarget")
  public boolean isAtTarget() {
    return this.IO.isAtTarget();
  }

  public void updateMech2D() {
    // 0 deg points up in Mech2d, +90 points left (or back in 3d)
    this.mech2d.setAngle((360 - (this.inputs.angleDegrees)) - 270);
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
        ShoulderConstants.kInitialPose
            .transformBy(RobotContainer.elevator.transform)
            .transformBy(this.transform);
  }
}
