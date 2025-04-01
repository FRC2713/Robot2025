package frc.robot.subsystems.pivot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.constants.PivotConstants;
import frc.robot.util.LoggedTunableGains;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Pivot extends SubsystemBase {
  private final PivotInputsAutoLogged inputs;
  private final PivotIO IO;
  public final MechanismLigament2d mech2d =
      new MechanismLigament2d(
          "pivot",
          PivotConstants.kLength,
          PivotConstants.kInitialAngleRad,
          PivotConstants.mech2dWidth,
          PivotConstants.mech2dColor);

  public Pose3d pose = PivotConstants.kInitialPose;
  public Transform3d transform = new Transform3d();

  public Pivot(PivotIO IO) {
    this.inputs = new PivotInputsAutoLogged();
    IO.updateInputs(inputs);
    this.IO = IO;
  }

  public void periodic() {
    if (PivotConstants.Gains.hasChanged(hashCode())) {
      this.IO.setPID(PivotConstants.Gains);
    }

    IO.updateInputs(inputs);
    Logger.processInputs("Pivot", inputs);

    this.transform = PivotConstants.kInitialTransform;

    Pose3d pivotPoint = RobotContainer.shoulder.pose.transformBy(this.transform);

    this.pose =
        new Pose3d(pivotPoint.getTranslation(), new Rotation3d())
            .transformBy(
                new Transform3d(
                    new Translation3d(),
                    new Rotation3d(0, Units.degreesToRadians(inputs.angleDegrees), 0)));
  }

  public void setTargetAngle(double degrees) {
    this.IO.setTargetAngle(degrees);
  }

  @AutoLogOutput(key = "Pivot/isAtTarget")
  public boolean isAtTarget() {
    return this.IO.isAtTarget();
  }

  public void updateMech2D() {
    // 0 deg points up in Mech2d, +90 points left (or back in 3d)
    this.mech2d.setAngle((270 - this.inputs.angleDegrees));
  }

  public double getCurrentAngle() {
    return this.inputs.angleDegrees;
  }

  public void setBus(double bus) {
    Logger.recordOutput("bus", bus);
    IO.setBus(bus);
  }

  public void setPID(LoggedTunableGains pid) {
    this.IO.setPID(pid);
  }
}
