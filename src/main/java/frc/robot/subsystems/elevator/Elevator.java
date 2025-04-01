package frc.robot.subsystems.elevator;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.constants.ElevatorConstants;
import frc.robot.util.LoggedTunableGains;
import org.littletonrobotics.junction.AutoLogOutput;
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

  public Pose3d pose = ElevatorConstants.kInitialPose;
  public Transform3d transform = new Transform3d();

  public Elevator(ElevatorIO IO) {
    this.inputs = new ElevatorInputsAutoLogged();
    this.IO = IO;
    this.IO.updateInputs(this.inputs);
  }

  @Override
  public void periodic() {
    if (ElevatorConstants.Gains.hasChanged(hashCode())) {
      this.IO.setPID(ElevatorConstants.Gains);
    }

    this.IO.updateInputs(this.inputs);
    Logger.processInputs("Elevator", this.inputs);

    this.transform =
        new Transform3d(0, 0, Units.inchesToMeters(getCurrentHeight()), new Rotation3d());

    this.pose = ElevatorConstants.kInitialPose.transformBy(this.transform);
  }

  public double getCurrentHeight() {
    return (inputs.heightInchesLeft + inputs.heightInchesRight) / 2;
  }

  public void setTargetHeight(double height) {
    this.IO.setTargetHeight(height);
  }

  public void setVoltage(double volts1, double volts2) {
    this.IO.setVoltage(volts1, volts2);
  }

  @AutoLogOutput(key = "Elevator/isAtTarget")
  public boolean isAtTarget() {
    return this.IO.isAtTarget();
  }

  public void updateMech2D() {
    this.mech2d.setLength(Units.inchesToMeters(this.inputs.heightInchesLeft));
  }

  public void setPID(LoggedTunableGains pid) {
    this.IO.setPID(pid);
  }
}
