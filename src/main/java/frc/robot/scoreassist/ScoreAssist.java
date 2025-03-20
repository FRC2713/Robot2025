package frc.robot.scoreassist;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.RobotContainer;
import frc.robot.subsystems.constants.ScoreAssistConstants;
import frc.robot.util.ScoreLevel;
import frc.robot.util.ScoreLoc;
import frc.robot.util.ScoreNode;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

/**
 * Manages subscribing to network tables UI and maintaining the most up-to-date pose and
 * superstructure target
 */
public class ScoreAssist {

  private final StringSubscriber reefTrackerSub =
      NetworkTableInstance.getDefault().getStringTopic("/scoreassist/goto").subscribe("none");

  @Getter private ScoreNode currentNodeTarget = ScoreNode.A;
  @Getter private ScoreLevel currentLevelTarget = ScoreLevel.FOUR;
  private boolean updatedNodeTarget = false;
  private boolean updatedLevelTarget = false;

  private double xError = Double.MAX_VALUE;
  private double yError = Double.MAX_VALUE;
  private double thetaError = Double.MAX_VALUE;
  private double pathTargetError = Double.MAX_VALUE;

  private Alert usingClosest = new Alert("Using Closest Node", AlertType.kWarning);

  public ScoreDrivingMode mode = ScoreDrivingMode.INACTIVE;

  public void periodic() {
    if (DriverStation.isTeleop()) {
      this.updateWithNT();
    }

    Pose2d targetToRobotError =
        this.currentNodeTarget
            .getRobotAlignmentPose()
            .relativeTo(RobotContainer.driveSubsystem.getPose());
    this.xError = targetToRobotError.getX();
    this.yError = targetToRobotError.getY();
    this.thetaError = targetToRobotError.getRotation().getDegrees();

    this.pathTargetError =
        this.currentNodeTarget
            .getPathScorePose()
            .getTranslation()
            .getDistance(RobotContainer.driveSubsystem.getTranslation());

    Logger.recordOutput("ScoreAssist/mode", this.mode);
    Logger.recordOutput(
        "ScoreAssist/currentNodeTarget", this.currentNodeTarget.getRobotAlignmentPose());
    Logger.recordOutput("ScoreAssist/currentLevelTarget", this.currentLevelTarget.name());
    Logger.recordOutput("ScoreAssist/xErrorInches", Units.metersToInches(this.xError));
    Logger.recordOutput("ScoreAssist/yErrorInches", Units.metersToInches(this.yError));
    Logger.recordOutput("ScoreAssist/thetaError", this.thetaError);
    Logger.recordOutput(
        "ScoreAssist/pathTargetErrorInches", Units.metersToInches(this.pathTargetError));
  }

  // ScoreAssist targets can update with a manually provided location
  public void updateManually(ScoreLoc commandedScoreLoc) {
    this.updatedLevelTarget = true;
    this.updatedNodeTarget = true;
    this.currentNodeTarget = commandedScoreLoc.getNode();
    this.currentLevelTarget = commandedScoreLoc.getLevel();

    Logger.recordOutput("ScoreAssist/updatedNodeTarget", updatedNodeTarget);
    Logger.recordOutput("ScoreAssist/updatedLevelTarget", updatedLevelTarget);
  }

  // ScoreAssist targets can update by taking in network table updates
  public void updateWithNT() {
    String reefTrackerInput = reefTrackerSub.get();
    var loc = ScoreLoc.parseFromNT(reefTrackerInput);
    Logger.recordOutput("ScoreAssist/reefTrackerInput", reefTrackerInput);
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

  // ScoreAssist targets can update by calculating the closest branch to the robot
  public void updateWithClosest() {
    var loc = ScoreLoc.parseFromNT(reefTrackerSub.get());
    if (loc != null) {
      this.updatedLevelTarget = this.currentLevelTarget != loc.getLevel();
      this.currentLevelTarget = loc.getLevel();
    }

    ScoreNode closestNode = this.getClosestNode();
    this.updatedNodeTarget = this.currentNodeTarget != closestNode;
    this.currentNodeTarget = closestNode;

    Logger.recordOutput("ScoreAssist/updatedNodeTarget", updatedNodeTarget);
    Logger.recordOutput("ScoreAssist/updatedLevelTarget", updatedLevelTarget);
  }

  public ScoreNode getClosestNode() {
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

  // ScoreAssist will be done "assisting" when the robot gets close enough
  public boolean driveAssistIsDone() {
    boolean slow = RobotContainer.driveSubsystem.getSpeed() < 0.08;
    boolean withinX = Math.abs(this.xError) < ScoreAssistConstants.assistXTolerance.getAsDouble();
    boolean withinY = Math.abs(this.yError) < ScoreAssistConstants.assistYTolerance.getAsDouble();
    boolean withinTheta =
        Math.abs(this.thetaError) < ScoreAssistConstants.assistThetaTolerance.getAsDouble();

    return this.mode == ScoreDrivingMode.ASSIST
        && currentNodeTarget != null
        && withinX
        && withinY
        && withinTheta
        && slow;
  }

  public boolean shouldUsePath() {
    return RobotContainer.driveSubsystem
            .getPose()
            .getTranslation()
            .getDistance(
                RobotContainer.scoreAssist
                    .getCurrentNodeTarget()
                    .getRobotAlignmentPose()
                    .getTranslation())
        > ScoreAssistConstants.pathDistTolerance.getAsDouble();
  }

  public boolean shouldManuallyOverridePath() {

    return Math.abs(Math.hypot(RobotContainer.driver.getLeftX(), RobotContainer.driver.getLeftY()))
        > 0.1;
  }

  public static enum ScoreDrivingMode {
    PATH,
    PATH_OVERRIDEN,
    ASSIST,
    INACTIVE
  }
}
