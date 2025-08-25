package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableGains;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

// Intake is essentially Shoulder and EndEfffector mashed into one class- one EndEffector roller +
// sholder for controling intake angle

public class Intake extends SubsystemBase {
  private final IntakeIO IO;
  private final IntakeInputsAutoLogged inputs = new IntakeInputsAutoLogged();

  @AutoLogOutput(key = "Intake/hadCoral")
  private boolean hadCoral = false;

  public Intake(IntakeIO IO) {
    this.IO = IO;
  }

  public void periodic() {
    Logger.processInputs("Intake", inputs);
    IO.updateInputs(inputs);

    var hasCoral = hasCoral();

    /*
    // Coppied & modified from EndEffector; uncomment if intake proves to move too quickly with coral
    if (hasCoral && !hadCoral) {
      System.out.println("Coral detected; slowing down intake arm");
      RobotContainer.intake.setPID(IntakeConstants.SlowGains);
    } else if (!hasCoral && hadCoral) {
      System.out.println("No Coral detected; Speeding up intake arm");
      RobotContainer.intake.setPID(IntakeConstants.Gains);
    }
    
    hadCoral = hasCoral;*/
  }

  //Roller functions

  public void setRollerRPM(double rpm) {
    IO.setRollerRPM(rpm);
  }

  public void setRollerCurrentLimit(int currentLimit) {
    IO.setRollerCurrentLimit(currentLimit);
  }

  public boolean hasCoral() {
    return inputs.hasCoral;
  }

  @AutoLogOutput(key = "Intake/rollerIsAtTarget")
  public boolean rollerIsAtTarget() {
    return this.IO.rollerIsAtTarget();
  }

  //Pivot functions
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
}
