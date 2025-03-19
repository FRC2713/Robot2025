package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.Drivetrain;
import frc.robot.util.ScoreLevel;
import frc.robot.util.ScoreLoc;
import frc.robot.util.ScoreNode;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class ReefTracker extends Command {

  private StringSubscriber sub =
      NetworkTableInstance.getDefault().getStringTopic("/scoreassist/goto").subscribe("none");
  @Getter private ScoreLoc reefTrackerLoc = null;
  private Alert alert = new Alert("Using closest location", AlertType.kWarning);

  private ScoreAssist scoreAsist = null;
  private PathScore pathScore;

  @Getter private boolean doSS = false;
  @Getter private boolean doScore = false;
  @Getter private boolean doPrep = false;

  enum ScoreMode {
    ASSIST,
    PATH,
    SWITCH_TO_ASSIST
  }

  private ScoreMode mode = ScoreMode.PATH;

  private Drivetrain drive;

  public ReefTracker(Drivetrain drive) {
    this.drive = drive;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    doPrep = false;
    doScore = false;
    doSS = false;
    var loc = ScoreLoc.parseFromNT(sub.get());
    if (loc != null) {
      reefTrackerLoc = loc;
      alert.set(false);
    } else {
      reefTrackerLoc = ScoreLoc.fromNodeAndLevel(getClosestNode(), ScoreLevel.ONE);
      alert.set(true);
    }
    Logger.recordOutput("ScoreAssist/reefTrackerLoc", reefTrackerLoc);
    scoreAsist = new ScoreAssist(reefTrackerLoc.getNode(), drive);
    pathScore = new PathScore(drive, reefTrackerLoc.getNode());
    if (shouldUseScoreAssist()) {
      mode = ScoreMode.SWITCH_TO_ASSIST;
    } else {
      pathScore.initialize();
      mode = ScoreMode.PATH;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Logger.recordOutput("ScoreAssist/Mode", mode);
    if (mode == ScoreMode.SWITCH_TO_ASSIST) {
      scoreAsist.initialize();
      mode = ScoreMode.ASSIST;
    }

    if (mode == ScoreMode.PATH) {
      pathScore.execute();
      if (pathScore.isFinished()) {
        mode = ScoreMode.SWITCH_TO_ASSIST;
      }
    }
    if (mode == ScoreMode.ASSIST) {
      scoreAsist.execute();
      if (scoreAsist.isReadyForScore()) {
        // if (!hasScored) {
        //   Commands.sequence(
        //           reefTrackerLoc.getLevel().getSsCommand().get(),
        //           Commands.print("SS Done! HERE! LOL!"),
        //           RollerCmds.setSpeed(SSConstants.Roller.L2_PLUS_CORAL_SCORE_SPEED),
        //           Commands.waitSeconds(SSConstants.Auto.L4_POST_SCORE_DELAY.getAsDouble()),
        //           RollerCmds.setSpeed(() -> 0))
        //       .schedule();
        doScore = true;
        // }
      } else if (scoreAsist.isReadyForScoreSS()) {
        // if (!hasScoredSS) {
        //   reefTrackerLoc.getLevel().getSsCommand().get().schedule();
        //   hasScoredSS = true;
        // }
        doSS = true;
      } else if (scoreAsist.isReadyForPrep()) {
        // if (!hasPrepped) {
        //   reefTrackerLoc.getLevel().getPrepCommand().get().schedule();
        //   hasPrepped = true;
        // }
        doPrep = true;
      }
    }
  }

  private static ScoreNode getClosestNode() {
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

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    scoreAsist.end(interrupted);
    pathScore.end(interrupted);
    super.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  private boolean shouldUseScoreAssist() {
    return drive
            .getPose()
            .getTranslation()
            .getDistance(reefTrackerLoc.getNode().getRobotAlignmentPose().getTranslation())
        < 1.25;
  }

  private static boolean shouldDoScoreAssist(ScoreNode node) {
    return RobotContainer.driveSubsystem
            .getPose()
            .getTranslation()
            .getDistance(node.getRobotAlignmentPose().getTranslation())
        < 1.25;
  }

  public static Command create(ScoreLoc loc) {
    return Commands.either(
        new ScoreAssist(loc.getNode(), RobotContainer.driveSubsystem),
        Commands.sequence(
            new PathScore(RobotContainer.driveSubsystem, loc.getNode()),
            new ScoreAssist(loc.getNode(), RobotContainer.driveSubsystem)),
        () -> shouldDoScoreAssist(loc.getNode()));
  }
}
