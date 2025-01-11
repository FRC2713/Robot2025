package frc.robot.subsystems.pivot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.constants.PivotConstants;
import org.littletonrobotics.junction.Logger;

public class Pivot extends SubsystemBase {
  private final PivotInputsAutoLogged inputs;
  private final PivotIO IO;
  public final MechanismLigament2d mech2d =
      new MechanismLigament2d(
          "pivot",
          PivotConstants.kLength,
          Units.degreesToRadians(PivotConstants.kInitialAngle),
          PivotConstants.mech2dWidth,
          PivotConstants.mech2dColor);

  private double targetAngleDeg = Units.degreesToRadians(PivotConstants.kInitialAngle);

  public Pivot(PivotIO IO) {
    this.inputs = new PivotInputsAutoLogged();
    IO.updateInputs(inputs);
    this.IO = IO;
  }

  public void periodic() {
    IO.updateInputs(inputs);
    Logger.processInputs("Pivot", inputs);
  }

  public void setTargetAngle(double degrees) {
    this.IO.setTargetAngle(degrees);
  }

  public boolean isAtTarget() {
    return Math.abs(this.inputs.angleDegrees - this.targetAngleDeg) < 0.01;
  }

  public void updateMech2D() {
    // 0 deg points up in Mech2d, +90 points left (or back in 3d)
    this.mech2d.setAngle((this.inputs.angleDegrees) - 90);
  }
}
