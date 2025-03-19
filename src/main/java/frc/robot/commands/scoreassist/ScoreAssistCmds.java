package frc.robot.commands.scoreassist;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.SuperStructure;
import frc.robot.scoreassist.ScoreAssist.ScoreDrivingMode;
import frc.robot.util.ScoreLoc;

public class ScoreAssistCmds {

  public static Command exectuteAllTargets() {
    return Commands.sequence(
        Commands.parallel(
            // 1) Activate
            start(),

            // 2) Drive to target in two parts
            Commands.sequence(
                // a) drive close to target with path finding
                executePath(),
                // b) drive to given target
                exectuteDrive()),
            // 3) move ss for given location
            RobotContainer.scoreAssist.getCurrentLevelTarget().getPrepCommand().get()),
        // 4) Finish ScoreAssist and Score!
        stop(),
        RobotContainer.scoreAssist.getCurrentLevelTarget().getScoreCommand().get(),
        SuperStructure.CORAL_SCORE.getCommand());
  }

  public static Command exectuteInAuto(ScoreLoc scoreLoc) {
    // Note that during autonomous, ScoreAssist does not update via NT
    return Commands.sequence(
        Commands.runOnce(() -> RobotContainer.scoreAssist.updateManually(scoreLoc)),
        Commands.parallel(
            // Activate
            start(),
            // Drive to given target
            exectuteDrive(),
            // move ss for given location
            RobotContainer.scoreAssist.getCurrentLevelTarget().getPrepCommand().get()),
        // 4) Finish ScoreAssist and Score!
        stop(),
        RobotContainer.scoreAssist.getCurrentLevelTarget().getScoreCommand().get(),
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
                () -> RobotContainer.scoreAssist.getCurrentNodeTarget()))
        .until(() -> RobotContainer.scoreAssist.drivePathIsDone());
  }

  public static Command exectuteDrive() {
    return Commands.sequence(
        Commands.runOnce(() -> RobotContainer.scoreAssist.mode = ScoreDrivingMode.ASSIST),
        new DriveToPose(
                () -> RobotContainer.scoreAssist.getCurrentNodeTarget().getRobotAlignmentPose(),
                RobotContainer.driveSubsystem)
            .until(() -> RobotContainer.scoreAssist.driveAssistIsDone()));
  }
}
