package frc.robot.subsystems.elevator;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.constants.ElevatorConstants;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {

  private ElevatorInputsAutoLogged inputs;
  private ElevatorIO IO;
  public final MechanismLigament2d mech2d =
      new MechanismLigament2d(
          "elevator",
          ElevatorConstants.kInitialHeight,
          90,
          ElevatorConstants.mech2dWidth,
          ElevatorConstants.mech2dColor);

  private double targetHeightInches = 0.0;

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
    this.targetHeightInches = height;
  }

  public boolean isAtTarget() {
    return Math.abs(this.inputs.heightInchesLeft - this.targetHeightInches)
            < ElevatorConstants.kAcceptablePositionErrorInches
        && Math.abs(this.inputs.heightInchesRight - this.targetHeightInches)
            < ElevatorConstants.kAcceptablePositionErrorInches;
  }

  public void updateMech2D() {
    this.mech2d.setLength(
        Units.inchesToMeters((this.inputs.heightInchesLeft + this.inputs.heightInchesRight) / 2));
  }
}
