package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.SSConstants;
import frc.robot.subsystems.drive.Drivetrain;
import frc.robot.util.ScoreLoc;
import frc.robot.util.ScoreNode;

public class AutoScore {
  private static ScoreLoc loc = ScoreLoc.A_FOUR;

  private static boolean shouldDoScoreAssist(ScoreNode node) {
    return RobotContainer.driveSubsystem
            .getPose()
            .getTranslation()
            .getDistance(node.getRobotAlignmentPose().getTranslation())
        < 1.25;
  }

  public AutoScore(Drivetrain drive) {}

  private Command scoreAssist() {
    var scoreAssist = new ScoreAssist(loc.getNode(), RobotContainer.driveSubsystem);
    return Commands.parallel(
        scoreAssist,
        Commands.sequence(
            new WaitUntilCommand(scoreAssist::isReadyForPrep),
            loc.getLevel().getPrepCommand().get(),
            new WaitUntilCommand(scoreAssist::isReadyForScoreSS),
            loc.getLevel().getSsCommand().get(),
            new WaitUntilCommand(scoreAssist::isReadyForScore),
            RollerCmds.setSpeed(SSConstants.Roller.L2_PLUS_CORAL_SCORE_SPEED)));
  }

  public Command getCommand() {
    return Commands.either(
        scoreAssist(),
        Commands.sequence(
            new PathScore(RobotContainer.driveSubsystem, loc.getNode()), scoreAssist()),
        () -> shouldDoScoreAssist(loc.getNode()));
  }
}
