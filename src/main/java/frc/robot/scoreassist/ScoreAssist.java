package frc.robot.scoreassist;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.RobotContainer;
import frc.robot.subsystems.constants.ScoreAssistConstants;
import frc.robot.util.ReefTracker;
import frc.robot.util.ScoreLevel;
import frc.robot.util.ScoreLoc;
import frc.robot.util.ScoreLoc.ScoreLocations;
import frc.robot.util.ScoreLocType;
import frc.robot.util.ScoreNode;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * Manages subscribing to network tables UI and maintaining the most up-to-date pose and
 * superstructure target
 */
public class ScoreAssist {
  @Getter private ScoreNode currentNodeTarget = ScoreNode.A;
  @Getter private ScoreLevel currentLevelTarget = ScoreLevel.FOUR;
  private boolean updatedNodeTarget = false;
  private boolean updatedLevelTarget = false;

  private Alert usingClosest = new Alert("Using Closest Node", AlertType.kWarning);

  public ScoreDrivingMode mode = ScoreDrivingMode.INACTIVE;

  public void periodic() {
    if (DriverStation.isTeleop()) {
      this.updateWithNT();
    }

    // this.recalculateErrors();

    // Logging
    Logger.recordOutput("ScoreAssist/mode", this.mode);
    Logger.recordOutput(
        "ScoreAssist/currentNodeTarget", this.currentNodeTarget.getRobotAlignmentPose());
    Logger.recordOutput("ScoreAssist/currentLevelTarget", this.currentLevelTarget.name());
  }

  /**
   * Provides a way to set the commanded location directly. However, if in teleop, this will be
   * overridden as soon as a valid command is set from ReefTracker
   *
   * @param commandedScoreLoc the commanded score location
   */
  public void updateManually(ScoreLoc commandedScoreLoc) {
    this.updatedLevelTarget = true;
    this.updatedNodeTarget = true;
    this.currentNodeTarget = commandedScoreLoc.getNode();
    this.currentLevelTarget = commandedScoreLoc.getLevel();

    Logger.recordOutput("ScoreAssist/updatedNodeTarget", updatedNodeTarget);
    Logger.recordOutput("ScoreAssist/updatedLevelTarget", updatedLevelTarget);
  }

  /**
   * ScoreAssist will target any valid scoring location provided over network tables from
   * ReefTracker. If nothing has come in from ReefTracker, the closest scoring location will be
   * choosen
   */
  private void updateWithNT() {
    var loc = ScoreLocations.fromMsg(ReefTracker.getInstance().getGotoLoc());
    if (loc == null) {
      updateWithClosest();
      usingClosest.set(true);
      return;
    }
    usingClosest.set(false);

    this.updatedNodeTarget = this.currentNodeTarget != loc.getNode();
    this.updatedLevelTarget = this.currentLevelTarget != loc.getLevel();

    this.currentLevelTarget = loc.getLevel();
    this.currentNodeTarget = loc.getNode();

    Logger.recordOutput("ScoreAssist/updatedNodeTarget", updatedNodeTarget);
    Logger.recordOutput("ScoreAssist/updatedLevelTarget", updatedLevelTarget);
  }

  /**
   * ScoreAssist will target the closest scoring node if nothing else has been specified from
   * ReefTracker. This does not effect the scoring level.
   */
  private void updateWithClosest() {
    ScoreNode closestNode = this.getClosestNode();
    this.updatedNodeTarget = this.currentNodeTarget != closestNode;
    this.currentNodeTarget = closestNode;

    Logger.recordOutput("ScoreAssist/updatedNodeTarget", updatedNodeTarget);
  }

  /** Helper function to determine what the closest scoring node is */
  private ScoreNode getClosestNode() {
    var pose = RobotContainer.driveSubsystem.getPose();
    ScoreNode closestNode = ScoreNode.A;
    double closestDistance = Double.MAX_VALUE;

    for (ScoreNode node : ScoreNode.values()) {
      double distance =
          pose.getTranslation().getDistance(node.getRobotAlignmentPose().getTranslation());
      if (distance < closestDistance && node.getType() == ScoreLocType.CORAL) {
        closestDistance = distance;
        closestNode = node;
      }
    }

    return closestNode;
  }

  /**
   * The path-portion of score assist has a specific target. When the drivetrain is close enough to
   * that target, score assist is done using paths and will beign using profiled pid
   *
   * @return if the path-portion is done (based on drivetrain position)
   */
  @AutoLogOutput(key = "ScoreAssist/isAtPathTargetPose")
  public boolean isAtPathTargetPose() {
    return RobotContainer.driveSubsystem
                .getPose()
                .getTranslation()
                .getDistance(
                    RobotContainer.scoreAssist
                        .getCurrentNodeTarget()
                        .getRobotAlignmentPose()
                        .getTranslation())
            > ScoreAssistConstants.pathDistTolerance.getAsDouble()
        && RobotContainer.autoScorePathing;
  }

  /**
   * Under certain circumstances, do not do the path-portion of score assist
   *
   * @return if we should be using the path-portion or letting the user drive
   */
  @AutoLogOutput(key = "ScoreAssist/shouldOverridePath")
  public boolean shouldOverridePath() {
    double translationStickMag =
        Math.abs(
            Math.hypot(
                RobotContainer.driverControls.getLeftX(),
                RobotContainer.driverControls.getLeftY()));

    return translationStickMag > 0.15;
  }

  public static enum ScoreDrivingMode {
    PATH,
    PATH_OVERRIDEN,
    ASSIST,
    BARGE,
    INACTIVE
  }
}
