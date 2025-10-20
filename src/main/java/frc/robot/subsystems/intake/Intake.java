package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.constants.IntakeConstants;
import frc.robot.util.LoggedTunableGains;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

// Intake is essentially Shoulder and EndEfffector mashed into one class- one EndEffector roller +
// sholder for controling intake angle

public class Intake extends SubsystemBase {
  private final IntakeIO IO;
  private final IntakeInputsAutoLogged inputs;
  public Pose3d pose = IntakeConstants.kInitialPose;
  public Transform3d transform = new Transform3d();

  @AutoLogOutput(key = "Intake/hadCoral")
  private boolean hadCoral = false;

  public Intake(IntakeIO IO) {
    this.inputs = new IntakeInputsAutoLogged();
    IO.updateInputs(inputs);
    this.IO = IO;
  }

  public void periodic() {

    IO.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
    Logger.recordOutput("doesThisWork", 42);
    updateTransform();

    @SuppressWarnings("unused")
    var hasCoral = hasObject();

    /*
    // Coppied & modified from EndEffector; uncomment if intake pivot proves to move too quickly with coral
    if (hasCoral && !hadCoral) {
      System.out.println("Coral detected; slowing down intake arm");
      RobotContainer.intake.setPID(IntakeConstants.SlowGains);
    } else if (!hasCoral && hadCoral) {
      System.out.println("No Coral detected; Speeding up intake arm");
      RobotContainer.intake.setPID(IntakeConstants.Gains);
    }

    hadCoral = hasCoral;*/
  }

  // Roller functions

  public boolean hasObject() {
    return inputs.hasObject;
  }

  public void setRollerVoltage(double volts) {
    IO.setRollerVoltage(volts);
  }

  // Pivot functions
  public void setTargetAngle(double degrees) {
    this.IO.setTargetAngle(degrees);
  }

  public void setPID(LoggedTunableGains gains) {
    this.IO.setPID(gains);
  }

  public double getCurrentAngle() {
    return this.inputs.intakePivotAngleDegrees;
  }

  public double getAbsoluteAngle() {
    return this.inputs.intakePivotAbsoluteAngleDegrees;
  }

  @AutoLogOutput(key = "Intake/intakePivotIsAtTarget")
  public boolean intakePivotIsAtTarget() {
    return this.IO.intakePivotIsAtTarget();
  }

  private void updateTransform() {
    this.transform =
        new Transform3d(
            0.0,
            0.0,
            0.0,
            new Rotation3d(0, Units.degreesToRadians(inputs.intakePivotAngleDegrees), 0));

    this.pose = IntakeConstants.kInitialPose.transformBy(this.transform);
  }
}
