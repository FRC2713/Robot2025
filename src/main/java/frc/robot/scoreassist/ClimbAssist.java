package frc.robot.scoreassist;

import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import org.littletonrobotics.junction.Logger;

/** Manages subscribing to network tables UI for climber prep command */
public class ClimbAssist {

  private final BooleanSubscriber climbPrepSub =
      NetworkTableInstance.getDefault().getBooleanTopic("/scoreassist/climbPrep").subscribe(false);

  private boolean lastCommand = false;
  private boolean updatedClimbCommand = false;

  public void periodic() {
    if (DriverStation.isTeleop()) {
      boolean recieved = climbPrepSub.get(false);
      if (recieved != this.lastCommand) {
        this.lastCommand = recieved;
        this.updatedClimbCommand = true;
      } else {
        this.updatedClimbCommand = false;
      }
    }

    Logger.recordOutput("ClimbAssist/command", this.lastCommand);
    Logger.recordOutput("ClimbAssist/updated", this.updatedClimbCommand);
  }

  public boolean shouldClimbPrep() {
    return this.lastCommand;
  }
}
