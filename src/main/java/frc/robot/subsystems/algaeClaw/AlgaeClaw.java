package frc.robot.subsystems.algaeClaw;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class AlgaeClaw extends SubsystemBase {

  private final AlgaeClawInputsAutoLogged inputs = new AlgaeClawInputsAutoLogged();
  private final AlgaeClawIO IO;

  public final MechanismLigament2d mech2d =
      new MechanismLigament2d(
          "algaeclaw", Units.inchesToMeters(2), -20, 7, new Color8Bit(100, 100, 100));

  public AlgaeClaw(AlgaeClawIO IO) {
    this.IO = IO;
  }

  public void periodic() {
    IO.updateInputs(this.inputs);
    Logger.processInputs("AlgaeClaw", inputs);
  }

  public void setRPM(double rpm) {
    IO.setRPM(rpm);
  }

  @AutoLogOutput(key = "AlgaeClaw/isAtTarget")
  public boolean isAtTarget() {
    return this.IO.isAtTarget();
  }

  public void updateMech2D() {
    mech2d.setAngle(Rotation2d.fromDegrees(inputs.algaeClawPositionDegs));
  }

  public boolean hasAlgae() {
    return inputs.hasAlgae;
  }

  public void setEnableLimitSwitch(boolean setEnable) {
    IO.setEnableLimitSwitch(setEnable);
  }
}
