package frc.robot.commands.scoreassist;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
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
               AllianceFlipUtil.apply(new Pose2d(
                    new Translation2d(0.4480087459087372, 1.305626749992370),
                    Rotation2d.fromRadians(0.996491486039043)))));
  }

  public static Command exectuteAllTargets() {
    return Commands.sequence(
        Commands.parallel(
            // 1) Activate
            start(),
            // 2) Drive to target in two parts
            Commands.either(
                Commands.sequence(
                    // a) drive close to target with path finding
                    executePath(),
                    // b) drive to given target
                    exectuteDrive()),
                exectuteDrive(),
                ScoreAssistCmds::shouldUsePath),
            // 3) move ss for given location
            new InstantCommand(
                () ->
                    RobotContainer.scoreAssist
                        .getCurrentLevelTarget()
                        .getPrepCommand()
                        .get()
                        .schedule())),
        // 4) Finish ScoreAssist and Score!
        stop(),
        new InstantCommand(
            () ->
                RobotContainer.scoreAssist
                    .getCurrentLevelTarget()
                    .getSsCommand()
                    .get()
                    .andThen(SuperStructure.CORAL_SCORE.getCommand())
                    .schedule()));
  }

  public static Command exectuteInAuto(ScoreLoc scoreLoc) {
    // Note that during autonomous, ScoreAssist does not update via NT
    return Commands.sequence(
        Commands.runOnce(() -> RobotContainer.scoreAssist.updateManually(scoreLoc)),
        Commands.parallel(
            // 1) Activate
            start(),
            // 2) Drive to given target
            exectuteDrive(),
            // 3) move ss for given location
            RobotContainer.scoreAssist.getCurrentLevelTarget().getPrepCommand().get()),
        // 4) Finish ScoreAssist and Score!
        stop(),
        RobotContainer.scoreAssist.getCurrentLevelTarget().getSsCommand().get(),
        SuperStructure.CORAL_SCORE.getCommand());
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
                RobotContainer.driveSubsystem)
            .until(() -> RobotContainer.scoreAssist.driveAssistIsDone()));
  }

  public static boolean shouldUsePath() {
    return RobotContainer.driveSubsystem
            .getPose()
            .getTranslation()
            .getDistance(
                RobotContainer.scoreAssist
                    .getCurrentNodeTarget()
                    .getRobotAlignmentPose()
                    .getTranslation())
        > 1.25;
  }
}
