package frc.robot.subsystems.pivot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.constants.PivotConstants;
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

  private double targetAngleDeg = Units.radiansToDegrees(PivotConstants.kInitialAngleRad);

  public Pivot(PivotIO IO) {
    this.inputs = new PivotInputsAutoLogged();
    IO.updateInputs(inputs);
    this.IO = IO;
  }

  public void periodic() {
    if (PivotConstants.PID.hasChanged(hashCode())) {
      this.IO.setPID(PivotConstants.PID);
    }

    IO.updateInputs(inputs);
    Logger.processInputs("Pivot", inputs);
  }

  public void setTargetAngle(double degrees) {
    this.targetAngleDeg = degrees;
    this.IO.setTargetAngle(degrees);
  }

  @AutoLogOutput(key = "Pivot/isAtTarget")
  public boolean isAtTarget() {
    return Math.abs(this.inputs.angleDegrees - this.targetAngleDeg)
        < PivotConstants.AT_TARGET_GIVE_DEGS;
  }

  public void updateMech2D() {
    // 0 deg points up in Mech2d, +90 points left (or back in 3d)
    this.mech2d.setAngle((this.inputs.angleDegrees) - 90);
  }

  public double getCurrentAngle() {
    return this.inputs.angleDegrees;
  }

  public void setBus(double bus) {
    Logger.recordOutput("bus", bus);
    IO.setBus(bus);
  }
}
