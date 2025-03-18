package frc.robot.scoreassist;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;
import frc.robot.subsystems.constants.ScoreAssistConstants;
import frc.robot.util.ScoreLevel;
import frc.robot.util.ScoreLoc;
import frc.robot.util.ScoreNode;
import lombok.Getter;

/**
 * Manages subscribing to network tables UI and maintaining the most up-to-date pose and
 * superstructure target
 */
public class ScoreAssist {

  private final StringSubscriber reefTrackerSub =
      NetworkTableInstance.getDefault().getStringTopic("/scoreassist/goto").subscribe("none");

  @Getter private ScoreNode currentNodeTarget;
  @Getter private ScoreLevel currentLevelTarget;
  private boolean updatedNodeTarget;
  private boolean updatedLevelTarget;

  public boolean isActive = false;

  public void periodic() {
    if (isActive && !DriverStation.isAutonomous()) {
      this.updateWithNT();
    }

    Logger.recordOutput("ScoreAssist/isActive", this.isActive);
    Logger.recordOutput("ScoreAssist/updatedNodeTarget", this.currentNodeTarget.getRobotAlignmentPose());
    Logger.recordOutput("ScoreAssist/updatedLevelTarget", this.currentLevelTarget.name());
    Logger.recordOutput("ScoreAssist/updatedNodeTarget", updatedNodeTarget);
    Logger.recordOutput("ScoreAssist/updatedLevelTarget", updatedLevelTarget);

  }

  public void updateManually(ScoreLoc commandedScoreLoc) {
    this.updatedLevelTarget = true;
    this.updatedNodeTarget = true;
    this.currentNodeTarget = commandedScoreLoc.getNode();
    this.currentLevelTarget = commandedScoreLoc.getLevel();
  }

  public void updateWithNT() {
    String reefTrackerInput = reefTrackerSub.get();
    var loc = ScoreLoc.parseFromNT(reefTrackerInput);
    Logger.recordOutput("ScoreAssist/reefTrackerInput", reefTrackerInput);
    if (loc == null) {
      return;
    }

    this.updatedNodeTarget = this.currentNodeTarget != loc.getNode();
    this.updatedLevelTarget = this.currentLevelTarget != loc.getLevel();

    this.currentLevelTarget = loc.getLevel();
    this.currentNodeTarget = loc.getNode();
  }

  public void updateWithClosest() {
    var loc = ScoreLoc.parseFromNT(reefTrackerSub.get());
    if (loc != null) {
      this.updatedLevelTarget = this.currentLevelTarget != loc.getLevel();
      this.currentLevelTarget = loc.getLevel();
    }

    ScoreNode closestNode = this.getClosestNode();
    this.updatedNodeTarget = this.currentNodeTarget != closestNode;
    this.currentNodeTarget = closestNode;
  }

  public Trigger nodeTrigger() {
    return new Trigger(() -> updatedNodeTarget && isActive);
  }

  public Trigger levelTrigger() {
    return new Trigger(() -> updatedLevelTarget && isActive);
  }

  private ScoreNode getClosestNode() {
    var pose = RobotContainer.driveSubsystem.getPose();
    ScoreNode closestNode = ScoreNode.A;
    double closestDistance = Double.MAX_VALUE;

    for (ScoreNode node : ScoreNode.values()) {
      double distance =
          pose.getTranslation().getDistance(node.getRobotAlignmentPose().getTranslation());
      if (distance < closestDistance) {
        closestDistance = distance;
        closestNode = node;
      }
    }

    return closestNode;
  }

  public boolean readyToAutoDrive() {
    // Only use score assist automated driving when close enough to targets
    return isActive
        && currentNodeTarget != null
        && RobotContainer.driveSubsystem
                .getTranslation()
                .getDistance(currentNodeTarget.getRobotAlignmentPose().getTranslation())
            < ScoreAssistConstants.scoreAsssistActivationThreshold.getAsDouble();
  }

  public boolean readyToAlignSS() {
    // Only align the super structure when the close enough to targets
    return isActive
        && currentNodeTarget != null
        && RobotContainer.driveSubsystem
                .getTranslation()
                .getDistance(currentNodeTarget.getRobotAlignmentPose().getTranslation())
            < ScoreAssistConstants.scoreAsssistTolerance.getAsDouble();
  }
}
