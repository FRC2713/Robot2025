package frc.robot.util;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringSubscriber;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class ReefTracker {
  private ReefTracker() {}

  @Getter private ScoreAssistMessage gotoLoc = null;

  public ScoreAssistMessage.GoalType getGoalTypeOrCoral() {
    return gotoLoc == null ? ScoreAssistMessage.GoalType.CORAL : gotoLoc.goal;
  }

  private final StringSubscriber reefTrackerSub =
      NetworkTableInstance.getDefault().getStringTopic("/scoreassist/goto").subscribe("none");

  public void periodic() {
    var fromNT = reefTrackerSub.get();
    Logger.recordOutput("ScoreAssist/fromNT", fromNT);
    try {
      gotoLoc = new ScoreAssistMessage(fromNT);
    } catch (Exception e) {
      gotoLoc = null;
    }
  }

  private static ReefTracker INSTANCE = null;

  public static ReefTracker getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new ReefTracker();
    }
    return INSTANCE;
  }
}
