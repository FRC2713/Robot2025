package frc.robot.subsystems.shoulder;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.constants.ShoulderConstants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Shoulder extends SubsystemBase {
  private final ShoulderInputsAutoLogged inputs;
  private final ShoulderIO IO;
  public final MechanismLigament2d mech2d =
      new MechanismLigament2d(
          "shoulder",
          ShoulderConstants.kLength,
          ShoulderConstants.kInitialAngleRad,
          ShoulderConstants.mech2dWidth,
          ShoulderConstants.mech2dColor);

  private double targetAngleDeg = Units.radiansToDegrees(ShoulderConstants.kInitialAngleRad);

  public Shoulder(ShoulderIO IO) {
    this.inputs = new ShoulderInputsAutoLogged();
    IO.updateInputs(inputs);
    this.IO = IO;
  }

  public void periodic() {
    if (ShoulderConstants.PID.hasChanged(hashCode())) {
      this.IO.setPID(ShoulderConstants.PID);
    }

    IO.updateInputs(inputs);
    Logger.processInputs("Shoulder", inputs);
  }

  public void setTargetAngle(double degrees) {
    this.targetAngleDeg = degrees;
    this.IO.setTargetAngle(degrees);
  }

  @AutoLogOutput(key = "Shoulder/isAtTarget")
  public boolean isAtTarget() {
    return Math.abs(this.inputs.angleDegrees - this.targetAngleDeg)
        < ShoulderConstants.AT_TARGET_GIVE_DEGS;
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
