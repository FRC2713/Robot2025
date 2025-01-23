package frc.robot.subsystems.elevator;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  public MechanismLigament2d mech2d = new MechanismLigament2d("elevator", 0, 90, 20, new Color8Bit(255, 255, 255));
  private ElevatorInputsAutoLogged inputs;
  private ElevatorIO IO;

  public Elevator(ElevatorIO IO) {
    this.inputs = new ElevatorInputsAutoLogged();
    this.IO = IO;
    this.IO.updateInputs(this.inputs);
  }

  @Override
  public void periodic() {
    this.IO.updateInputs(this.inputs);
    Logger.processInputs("Elevator", this.inputs);
  }

  public void setTargetHeight(double height) {
    this.IO.setTargetHeight(height);
  }

  public void setVoltage(double volts1, double volts2) {
    this.IO.setVoltage(volts1, volts2);
  }

  public boolean isAtTarget() {
    return this.IO.isAtTarget();
  }
  public void updateMech2D() {
    this.mech2d.setLength(Units.inchesToMeters(this.inputs.heightInchesLeft));
  }
}
