package frc.robot.subsystems.endEffector;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SetpointConstants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class EndEffector extends SubsystemBase {

  private final EndEffectorInputsAutoLogged inputs = new EndEffectorInputsAutoLogged();
  private final EndEffectorIO IO;

  @AutoLogOutput(key = "EndEffector/hadAlgae")
  private boolean hadAlgae = false;

  public EndEffector(EndEffectorIO IO) {
    this.IO = IO;
  }

  public void periodic() {
    IO.updateInputs(this.inputs);
    Logger.processInputs("EndEffector", inputs);

    var hasAlgae = hasAlgae();
    if (hasAlgae && !hadAlgae) {
      System.out.println("Algae detected; slowing down SS");
      // RobotContainer.shoulder.setPID(ShoulderConstants.SlowGains);
      // RobotContainer.pivot.setPID(PivotConstants.SlowGains);
      // RobotContainer.elevator.setPID(ElevatorConstants.SlowGains);
    } else if (!hasAlgae && hadAlgae) {
      System.out.println("No algae detected; Speeding up SS");
      // RobotContainer.shoulder.setPID(ShoulderConstants.Gains);
      // RobotContainer.pivot.setPID(PivotConstants.Gains);
      // RobotContainer.elevator.setPID(ElevatorConstants.Gains);
    }
    hadAlgae = hasAlgae;
  }

  public void setCoralRPM(double rpm) {
    IO.setCoralRPM(rpm);
  }

  public void setAlgaeRPM(double rpm) {
    if (rpm < 0) {
      IO.setAlgaeRPM(rpm);
    } else if (hasAlgae()) {
      IO.setAlgaeRPM(SetpointConstants.AlgaeClaw.ALGAE_HOLD_SPEED.getAsDouble());
    } else {
      IO.setAlgaeRPM(rpm);
    }
  }

  @AutoLogOutput(key = "EndEffector/isCoralAtTarget")
  public boolean isCoralAtTarget() {
    return this.IO.isCoralAtTarget();
  }

  @AutoLogOutput(key = "EndEffector/isAlgaeAtTarget")
  public boolean isAlgaeAtTarget() {
    return this.IO.isAlgaeAtTarget();
  }

  public boolean hasCoral() {
    return inputs.hasCoral;
  }

  public boolean hasAlgae() {
    return inputs.hasAlgae;
  }

  public void setEnableLimitSwitch(boolean setEnable) {
    IO.setEnableLimitSwitch(setEnable);
  }

  public void setCoralCurrentLimit(int currentLimit) {
    IO.setCoralCurrentLimit(currentLimit);
  }

  public void setAlgaeCurrentLimit(int algaeCurrentLimit) {
    IO.setAlgaeCurrentLimit(algaeCurrentLimit);
  }
}
