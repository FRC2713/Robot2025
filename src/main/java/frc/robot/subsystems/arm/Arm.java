package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.SetpointConstants;
import frc.robot.subsystems.constants.ArmConstants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
  private final ArmInputsAutoLogged inputs;
  private final ArmIO IO;
  public Pose3d pose = ArmConstants.kInitialPose;
  public Transform3d transform = new Transform3d();

  public Arm(ArmIO IO) {
    this.inputs = new ArmInputsAutoLogged();
    IO.updateInputs(inputs);
    this.IO = IO;
  }

  public void periodic() {

    if (ArmConstants.Gains.hasChanged(hashCode())) {
      this.IO.setPID(ArmConstants.Gains);
    }

    IO.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);
    this.updateTransform();
  }

  public void setTargetAngle(double degrees) {
    this.IO.setTargetAngle(degrees);
  }

  @AutoLogOutput(key = "Arm/isAtTarget")
  public boolean isAtTarget() {
    return this.IO.isAtTarget();
  }

  public double getCurrentAngle() {
    return this.inputs.angleDegrees;
  }

  public void setBus(double bus) {
    Logger.recordOutput("bus", bus);
    IO.setBus(bus);
  }

  public void setCoralRPM(double rpm) {
    IO.setCoralRPM(rpm);
  }

  public void setAlgaeRPM(double rpm) {
    if (rpm < 0) {
      IO.setAlgaeRPM(rpm);
    } else if (handHasAlgae()) {
      IO.setAlgaeRPM(SetpointConstants.AlgaeClaw.ALGAE_HOLD_SPEED.getAsDouble());
    } else {
      IO.setAlgaeRPM(rpm);
    }
  }

  @AutoLogOutput(key = "Arm/isCoralAtTarget")
  public boolean handIsCoralAtTarget() {
    return this.IO.handIsCoralAtTarget();
  }

  @AutoLogOutput(key = "Arm/isAlgaeAtTarget")
  public boolean handIsAlgaeAtTarget() {
    return this.IO.isAlgaeAtTarget();
  }

  // figure this out when we know how to differentiate

  public boolean handHasCoral() {
    return inputs.hasCoral;
  }

  public boolean handHasAlgae() {
    return inputs.hasAlgae;
  }

  public void handSetEnableLimitSwitch(boolean setEnable) {
    IO.handSetEnableLimitSwitch(setEnable);
  }

  public void handSetCoralCurrentLimit(int currentLimit) {
    IO.handSetCoralCurrentLimit(currentLimit);
  }

  public void handSetAlgaeCurrentLimit(int algaeCurrentLimit) {
    IO.handSetAlgaeCurrentLimit(algaeCurrentLimit);
  }

  private void updateTransform() {
    this.transform =
        new Transform3d(
            0.0, 0.0, 0.0, new Rotation3d(0, Units.degreesToRadians(inputs.angleDegrees), 0));

    this.pose =
        ArmConstants.kInitialPose
            .transformBy(RobotContainer.elevator.transform)
            .transformBy(this.transform);
  }
}
