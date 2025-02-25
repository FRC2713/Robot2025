package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.networktables.StringTopic;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.ScoreLevel;
import frc.robot.util.ScoreLoc;
import frc.robot.util.ScoreNode;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class ScoreAssist {
  private Command activeCmd = null;
  private NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private StringTopic topic = inst.getStringTopic("/scoreassist/goto");
  private StringSubscriber sub;

  private static ScoreAssist INSTANCE = null;

  public static ScoreAssist getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new ScoreAssist();
    }
    return INSTANCE;
  }

  private ScoreAssist() {
    sub = topic.subscribe("none");
  }

  public Command setActiveCommand(Supplier<Command> cmd) {
    return new InstantCommand(
        () -> {
          cancel();
          activeCmd = cmd.get();
          schedule();
        });
  }

  public void schedule() {
    if (activeCmd != null) {
      activeCmd.schedule();
    }
  }

  public void cancel() {
    if (activeCmd != null) {
      activeCmd.cancel();
      activeCmd = null;
    }
  }

  public Command cancelCmd() {
    return new InstantCommand(() -> cancel());
  }

  public static Command getClosestCommand(Supplier<Pose2d> pose, ScoreLevel level) {
    Pose2d currentPose = pose.get();

    ScoreNode closestLoc = ScoreNode.A;
    double closestDist =
        currentPose.getTranslation().getDistance(closestLoc.getPose().getTranslation());

    for (ScoreNode loc : ScoreNode.values()) {
      double dist =
          currentPose
              .getTranslation()
              .getDistance(AllianceFlipUtil.apply(loc.getPose().getTranslation()));
      if (dist < closestDist) {
        closestLoc = loc;
        closestDist = dist;
      }
    }

    var closest = ScoreLoc.fromNodeAndLevel(closestLoc, level);
    Logger.recordOutput("/ScoreAssit/Closest", AllianceFlipUtil.apply(closestLoc.getPose()));
    if (closestLoc.getPose().getTranslation().getDistance(pose.get().getTranslation())
        < Units.inchesToMeters(0.1)) {
      Logger.recordOutput("/ScoreAssist/Mode", "Heading controller");
      return headingControllerDrive(closestLoc.getPose());
    }
    Logger.recordOutput("/ScoreAssist/Mode", "Pathing");
    return closest.getScoreCommand();
  }

  public static Command headingControllerDrive(Pose2d pose) {
    return DriveCommands.changeDefaultDriveCommand(
        RobotContainer.driveSubsystem,
        DriveCommands.joystickDriveAtAngle(
            RobotContainer.driveSubsystem, () -> 0, () -> 0, () -> pose.getRotation()),
        "Score Assist Heading Controller");
  }

  public Trigger getTrigger() {
    return new Trigger(() -> ScoreLoc.checkNTValid(sub.get("none")));
  }

  public Command networkTablesDrive() {
    return ScoreLoc.parseFromNT(sub.get("none")).getScoreCommand();
  }
}
