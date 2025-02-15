package frc.robot.subsystems.rollers;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.constants.RollerConstants;
import org.littletonrobotics.junction.Logger;

public class Rollers extends SubsystemBase {

  private final RollersInputsAutoLogged inputs = new RollersInputsAutoLogged();
  private final RollersIO IO;

  public final MechanismLigament2d mech2dTube =
      new MechanismLigament2d(
          "tuberollers", Units.inchesToMeters(2), -20, 7, new Color8Bit(100, 100, 100));

  private double tubeSpeedTarget;

  public Rollers(RollersIO IO) {
    this.IO = IO;
  }

  public void periodic() {
    IO.updateInputs(this.inputs);
    Logger.processInputs("Rollers", inputs);
  }

  public void setTubeRPM(double rpm) {
    tubeSpeedTarget = rpm;
    IO.setTubeRPM(rpm);
  }

  public boolean isTubeAtTarget() {
    return Math.abs(tubeSpeedTarget - inputs.tubeVelocityRPM) < RollerConstants.AT_TARGET_GIVE_RPM;
  }

  public void updateMech2D() {
    mech2dTube.setAngle(Rotation2d.fromDegrees(inputs.tubePositionDegs));
  }

  public boolean hasCoral() {
    return inputs.hasCoral;
  }

  public boolean hasAlgae() {
    return inputs.hasAlgae;
  }

  public void setEnableLimitSwitch(boolean setEnable) {
    IO.setEnableLimitSwitch(setEnable);
  }

  public void setEnableAlgaeLimitSwitch(boolean setEnable) {
    IO.setEnableAlgaeLimitSwitch(setEnable);
  }

  public void setCurrentLimit(double amps) {
    IO.setCurrentLimit(amps);
  }
}
