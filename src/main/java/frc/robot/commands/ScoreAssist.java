package frc.robot.commands;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
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

    final var closest = ScoreLoc.fromNodeAndLevel(closestLoc, level);
    Logger.recordOutput("/ScoreAssit/Closest", AllianceFlipUtil.apply(closestLoc.getPose()));
    Logger.recordOutput("/ScoreAssit/ClosestDist", closestDist);
    if (closestDist < Units.inchesToMeters(30)) {
      Logger.recordOutput("/ScoreAssist/Mode", "Align to tag");
      return alignToTagDrive(closest.getNode());
    }
    Logger.recordOutput("/ScoreAssist/Mode", "Pathing");
    return closest.getScoreCommand();
  }

  private static PIDController tagDrivePIDX = new PIDController(1, 0, 0);
  private static PIDController tagDrivePIDArea = new PIDController(1, 0, 0);

  private static Pair<Double, Double> getAlignToTagDriveOutput(ScoreNode node) {
    double targetX = node.isRightNode() ? 100 : 10;
    double targetArea = 50_000;
    var detections = RobotContainer.visionDetector.detections;
    System.out.println(detections);
    long targetId = node.getAprilTagID();
    if (detections.containsKey(targetId)) {
      var measurementX = detections.get(targetId).cx();
      var measurementArea = detections.get(targetId).area();
      var outputX = tagDrivePIDX.calculate(measurementX, targetX);
      var outputY = tagDrivePIDArea.calculate(measurementArea, targetArea);

      Logger.recordOutput("ScoreAssist/AlignToTag/Reasoning", "All good!");
      Logger.recordOutput("ScoreAssist/AlignToTag/outputX", outputX);
      Logger.recordOutput("ScoreAssist/AlignToTag/outputY", outputY);

      return new Pair<>(outputX, outputY);
    }

    Logger.recordOutput("ScoreAssist/AlignToTag/Reasoning", "Tag not seen");
    return new Pair<>(0., 0.);
  }

  public static Command alignToTagDrive(ScoreNode node) {
    return DriveCommands.changeDefaultDriveCommand(
        RobotContainer.driveSubsystem,
        DriveCommands.joystickDriveAtAngleRotationRelative(
            RobotContainer.driveSubsystem,
            () -> getAlignToTagDriveOutput(node),
            () -> AllianceFlipUtil.apply(node.getPose().getRotation())),
        "Align to Tag");
  }

  public Trigger getTrigger() {
    return new Trigger(() -> ScoreLoc.checkNTValid(sub.get("none")));
  }

  public Command networkTablesDrive() {
    return ScoreLoc.parseFromNT(sub.get("none")).getScoreCommand();
  }
}
