package frc.robot.commands.scoreassist;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.MoveSSToTarget;
import frc.robot.commands.SuperStructure;
import frc.robot.scoreassist.ScoreAssist.ScoreDrivingMode;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.ScoreLoc;

public class ScoreAssistCmds {

  public static Command intake() {
    return Commands.parallel(
        SuperStructure.SOURCE_CORAL_INTAKE.getCommand(),
        new PathfindToPose(
            RobotContainer.driveSubsystem,
            () ->
                AllianceFlipUtil.apply(
                    new Pose2d(
                        new Translation2d(0.4480087459087372, 1.305626749992370),
                        Rotation2d.fromRadians(0.996491486039043)))));
  }

  public static Command exectuteCoralScore() {
    // TODO: Use driver input as whether or not to fully automate
    return Commands.sequence(
            Commands.parallel(
                start(), // 1) activate score assist
                Commands.either(
                    Commands.sequence(
                        executePath(), // 2a) path-find close to target
                        exectuteDrive() // 2b) drive to target
                        ),
                    exectuteDrive(),
                    RobotContainer.scoreAssist::shouldUsePath),
                executePrep()), // 3) execute prep
            stop(),
            executeSS())
        .finallyDo(() -> RobotContainer.scoreAssist.mode = ScoreDrivingMode.INACTIVE);
  }

  public static Command exectuteCoralScoreInAuto(ScoreLoc scoreLoc) {
    // Note that during autonomous, ScoreAssist does not update via NT
    return Commands.sequence(
            Commands.runOnce(() -> RobotContainer.scoreAssist.updateManually(scoreLoc)),
            Commands.parallel(
                start(), // 1) activate score assist
                exectuteDrive(), // 2) drive to target
                executePrep()), // 3) execute prep
            stop(),
            executeSS())
        .finallyDo(() -> RobotContainer.scoreAssist.mode = ScoreDrivingMode.INACTIVE);
  }

  public static Command start() {
    return new InstantCommand(() -> RobotContainer.scoreAssist.mode = ScoreDrivingMode.PATH);
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

  public static Command executePath() {
    return Commands.sequence(
        Commands.runOnce(() -> RobotContainer.scoreAssist.mode = ScoreDrivingMode.PATH),
        new PathfindToPose(
            RobotContainer.driveSubsystem,
            () -> RobotContainer.scoreAssist.getCurrentNodeTarget().getPathScorePose()));
  }

  public static Command exectuteDrive() {
    return Commands.sequence(
        Commands.runOnce(() -> RobotContainer.scoreAssist.mode = ScoreDrivingMode.ASSIST),
        new DriveToPose(
            () -> RobotContainer.scoreAssist.getCurrentNodeTarget().getRobotAlignmentPose(),
            RobotContainer.driveSubsystem));
  }

  public static Command executePrep() {
    return new MoveSSToTarget(RobotContainer.scoreAssist::getCurrentLevelTarget);
  }

  public static Command executeSS() {
    return new MoveSSToTarget(
        RobotContainer.scoreAssist::getCurrentLevelTarget, SuperStructure.CORAL_SCORE.getCommand());
  }
}
