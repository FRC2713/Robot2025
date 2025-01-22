package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {

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
}
