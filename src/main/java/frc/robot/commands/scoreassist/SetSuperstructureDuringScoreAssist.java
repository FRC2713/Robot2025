package frc.robot.commands.scoreassist;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.scoreassist.ScoreAssist;
import frc.robot.util.ScoreLevel;
import java.util.function.Supplier;

public class SetSuperstructureDuringScoreAssist extends Command {

  ScoreAssist reefTracker;
  Supplier<ScoreLevel> ssLevel;
  boolean waitingForDrive = true;

  public SetSuperstructureDuringScoreAssist(
      ScoreAssist reefTracker, Supplier<ScoreLevel> ssLevel, SubsystemBase... requirements) {
    this.addRequirements(requirements);
    this.reefTracker = reefTracker;
    this.ssLevel = ssLevel;
  }

  @Override
  public void initialize() {
    ssLevel.get().getPrepCommand().get().schedule();
  }

  public void execute() {
    if (reefTracker.readyToAlignSS()) {
      ssLevel.get().getScoreCommand().get().schedule();
      waitingForDrive = false;
    }
  }

  public boolean isFinished() {
    return !waitingForDrive;
  }
}
