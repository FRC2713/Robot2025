package frc.robot.subsystems.algaeClaw;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.constants.RollerConstants;
import org.littletonrobotics.junction.Logger;

public class AlgaeClaw extends SubsystemBase {

  private final AlgaeClawInputsAutoLogged inputs = new AlgaeClawInputsAutoLogged();
  private final AlgaeClawIO IO;

  public final MechanismLigament2d mech2dTube =
      new MechanismLigament2d(
          "tubealgaeclaw", Units.inchesToMeters(2), -20, 7, new Color8Bit(100, 100, 100));

  private double tubeSpeedTarget;

  public AlgaeClaw(AlgaeClawIO IO) {
    this.IO = IO;
  }

  public void periodic() {
    IO.updateInputs(this.inputs);
    Logger.processInputs("AlgaeClaw", inputs);
  }

  public void setRPM(double rpm) {
    tubeSpeedTarget = rpm;
    IO.setRPM(rpm);
  }

  public boolean isAtTarget() {
    return Math.abs(tubeSpeedTarget - inputs.algaeClawVelocityRPM)
        < RollerConstants.AT_TARGET_GIVE_RPM;
  }

  public void updateMech2D() {
    mech2dTube.setAngle(Rotation2d.fromDegrees(inputs.algaeClawPositionDegs));
  }

  public boolean hasAlgae() {
    return inputs.hasAlgae;
  }

  public void setEnableLimitSwitch(boolean setEnable) {
    IO.setEnableLimitSwitch(setEnable);
  }
}
