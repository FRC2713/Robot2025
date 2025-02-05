package frc.robot.subsystems.rollers;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Rollers extends SubsystemBase {

  private final RollersInputsAutoLogged inputs = new RollersInputsAutoLogged();
  private final RollersIO IO;

  public final MechanismLigament2d mech2dTube =
      new MechanismLigament2d(
          "tuberollers", Units.inchesToMeters(5), -20, 7, new Color8Bit(100, 100, 100));
  public final MechanismLigament2d mech2dAlgae =
      new MechanismLigament2d(
          "algaerollers", Units.inchesToMeters(5), 20, 7, new Color8Bit(100, 100, 100));

  // NOTE: right now, controlling the tube is via the algae motor but in reverse
  // That is implemented in the HW/SIM layer
  private double tubeSpeedTarget;
  private double algaeSpeedTarget;

  public Rollers(RollersIO IO) {
    this.IO = IO;
  }

  public void periodic() {
    this.IO.updateInputs(this.inputs);
    Logger.processInputs("Rollers", inputs);
  }

  public void setAlgaeRPM(double rpm) {
    this.IO.setAlgaeRPM(rpm);
  }

  public void setTubeRPM(double rpm) {
    this.IO.setTubeRPM(rpm);
  }

  public boolean isTubeAtTarget() {
    return Math.abs(this.tubeSpeedTarget - this.inputs.tubeVelocityRPM) < 0.01;
  }

  public boolean isAlgaeAtTarget() {
    return Math.abs(this.algaeSpeedTarget - this.inputs.algaeVelocityRPM) < 0.01;
  }

  public void updateMech2D() {
    int tubeR = (int) Math.abs(Math.min((this.inputs.tubeOutputVoltage / 12.0), 0)) * 255;
    int tubeG = (int) Math.abs(Math.max((this.inputs.tubeOutputVoltage / 12.0), 0)) * 255;
    Color8Bit tubeColor = new Color8Bit(tubeR, tubeG, 0);
    int algaeR = (int) Math.abs(Math.min((this.inputs.algaeOutputVoltage / 12.0), 0)) * 255;
    int algaeG = (int) Math.abs(Math.max((this.inputs.algaeOutputVoltage / 12.0), 0)) * 255;
    Color8Bit algaeColor = new Color8Bit(algaeR, algaeG, 0);

    if (tubeR + tubeG < 100) {
      tubeColor = new Color8Bit(100, 100, 100);
    }
    if (algaeR + algaeG < 100) {
      algaeColor = new Color8Bit(100, 100, 100);
    }

    this.mech2dTube.setColor(tubeColor);
    this.mech2dAlgae.setColor(algaeColor);
  }

  public boolean hasCoral() {
    return this.IO.hasCoral();
  }
}
