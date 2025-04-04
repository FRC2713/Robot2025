package frc.robot.subsystems.climber;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.constants.ClimberConstants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {
  private final ClimberInputsAutoLogged inputs;
  private final ClimberIO IO;
  public final MechanismLigament2d mech2d =
      new MechanismLigament2d(
          "climbermech",
          ClimberConstants.kLength,
          ClimberConstants.kInitialAngle,
          10,
          new Color8Bit(255, 0, 0));

  public Pose3d pose = ClimberConstants.kInitialPose;
  public Transform3d transform = new Transform3d();

  public Climber(ClimberIO IO) {
    this.inputs = new ClimberInputsAutoLogged();
    IO.updateInputs(inputs);
    this.IO = IO;
  }

  public void periodic() {
    if (ClimberConstants.Gains.hasChanged(hashCode())) {
      this.IO.setPID(ClimberConstants.Gains);
    }

    IO.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);

    this.transform =
        new Transform3d(
            0, 0, 0, new Rotation3d(Units.degreesToRadians(inputs.leftAngleDegrees), 0, 0));

    this.pose = ClimberConstants.kInitialPose.transformBy(this.transform);
  }

  public void setTargetAngle(double degrees) {
    this.IO.setTargetAngle(degrees);
  }

  @AutoLogOutput(key = "Climber/isAtTarget")
  public boolean isAtTarget() {
    return this.IO.isAtTarget();
  }

  public void updateMech2D() {
    // 0 deg points up in Mech2d, +90 points left (or back in 3d)
    this.mech2d.setAngle((getCurrentAngle()) - 90);
  }

  public double getCurrentAngle() {
    return this.inputs.leftAngleDegrees;
  }

  public void setServoPos(double pos) {
    IO.setServoPos(pos);
  }

  public void setVoltage(double volts) {
    IO.setVoltage(volts);
  }

  public void configureSoftLimits(double minDeg, double maxDeg) {
    IO.configureSoftLimits(minDeg, maxDeg);
  }
}
