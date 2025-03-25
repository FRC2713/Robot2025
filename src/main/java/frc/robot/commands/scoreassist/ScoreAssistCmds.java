package frc.robot.commands.scoreassist;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.MoveSSToTarget;
import frc.robot.commands.superstructure.EndEffector;
import frc.robot.commands.superstructure.SuperStructure;
import frc.robot.scoreassist.ScoreAssist.ScoreDrivingMode;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.ScoreLoc;

public class ScoreAssistCmds {

  /** This moves the Superstructure and drives */
  public static Command intake() {
    return Commands.parallel(
        SuperStructure.SOURCE_CORAL_INTAKE.get(),
        new PathfindToPose(
            RobotContainer.driveSubsystem,
            () ->
                AllianceFlipUtil.apply(
                    new Pose2d(
                        new Translation2d(0.4480087459087372, 1.305626749992370),
                        Rotation2d.fromRadians(0.996491486039043)))));
  }

  /** This drives to target, moves the SS when ready, and runs the rollers when ready */
  public static Command exectuteCoralScore() {
    return Commands.sequence(
            // 1) activate score assist
            start(),
            // 2) drive to target and prep ss
            Commands.parallel(executeTeleopDrivingSequence(false), executePrep()),
            pause(ScoreDrivingMode.SCORING),
            // 3) score, keep adjusting drive position though
            Commands.race(exectutePID(true), executeSS()),
            // 4) stop score assist
            stop())
        .finallyDo(() -> RobotContainer.scoreAssist.mode = ScoreDrivingMode.INACTIVE);
  }

  /** This drives to target, moves the SS when ready, and runs the rollers when ready */
  public static Command exectuteCoralScoreInAuto(ScoreLoc scoreLoc) {
    // Note that during autonomous, ScoreAssist does not update via NT
    return Commands.sequence(
            // 1) set the target based on input and activate score assist
            Commands.runOnce(() -> RobotContainer.scoreAssist.updateManually(scoreLoc)),
            start(),
            // 2) drive to target and prep ss
            Commands.parallel(exectutePID(false), executePrep()),
            pause(ScoreDrivingMode.SCORING),
            // 3) score, keep adjusting drive position though
            Commands.race(exectutePID(true), executeSS()),
            // 4) stop score assist
            stop())
        .finallyDo(() -> RobotContainer.scoreAssist.mode = ScoreDrivingMode.INACTIVE);
  }

  public static Command start() {
    return new InstantCommand(() -> RobotContainer.scoreAssist.mode = ScoreDrivingMode.PATH);
  }

  private static Command pause(ScoreDrivingMode pausedMode) {
    return Commands.sequence(
        new InstantCommand(() -> RobotContainer.scoreAssist.mode = pausedMode),
        Commands.runOnce(
            () -> RobotContainer.driveSubsystem.stop(), RobotContainer.driveSubsystem));
  }

  public static Command stop() {
    return Commands.sequence(
        new InstantCommand(() -> RobotContainer.scoreAssist.mode = ScoreDrivingMode.INACTIVE),
        Commands.runOnce(
            () -> RobotContainer.driveSubsystem.stop(), RobotContainer.driveSubsystem));
  }

  public static Command manuallySetTarget(ScoreLoc target) {
    return new InstantCommand(() -> RobotContainer.scoreAssist.updateManually(target));
  }

  private static Command executeTeleopDrivingSequence(boolean driveInBackground) {
    // Chooses whether we actually need to start with a path or not
    return Commands.either(
        Commands.sequence(executePath(), exectutePID(driveInBackground)),
        exectutePID(driveInBackground),
        RobotContainer.scoreAssist::isAtPathTargetPose);
  }

  /** This drives to target */
  private static Command executePath() {
    return Commands.sequence(
        Commands.runOnce(() -> RobotContainer.scoreAssist.mode = ScoreDrivingMode.PATH),
        new PathfindToPose(
            RobotContainer.driveSubsystem,
            () -> RobotContainer.scoreAssist.getCurrentNodeTarget().getPathScorePose()));
  }

  /** This drives */
  private static Command exectutePID(boolean driveInBackground) {
    return Commands.sequence(
        Commands.runOnce(() -> RobotContainer.scoreAssist.mode = ScoreDrivingMode.ASSIST),
        new DriveToPose(
            () -> RobotContainer.scoreAssist.getCurrentNodeTarget().getRobotAlignmentPose(),
            RobotContainer.driveSubsystem,
            driveInBackground));
  }

  /** This moves the SS when ready */
  private static Command executePrep() {
    return new MoveSSToTarget(RobotContainer.scoreAssist::getCurrentLevelTarget);
  }

  /** This moves the SS when ready, and runs the rollers when ready */
  private static Command executeSS() {
    return new MoveSSToTarget(
        RobotContainer.scoreAssist::getCurrentLevelTarget,
        Commands.sequence(Commands.waitSeconds(0.1), EndEffector.CORAL_SCORE.get()));
  }
}
