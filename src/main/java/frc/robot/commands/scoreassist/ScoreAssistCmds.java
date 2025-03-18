package frc.robot.commands.scoreassist;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.util.ScoreLoc;

public class ScoreAssistCmds {

  public static Command exectuteAllTargets(Command defaultCmd) {
    return Commands.parallel(
        // activate
        activate(),
        // drive to given target
        exectuteDrive(defaultCmd),
        // move ss for given location
        executeSS());
  }

  public static Command activate() {
    return new InstantCommand(() -> RobotContainer.scoreAssist.isActive = true);
  }

  public static Command deactivate() {
    return new InstantCommand(() -> RobotContainer.scoreAssist.isActive = false);
  }

  public static Command manuallySetTarget(ScoreLoc target) {
    return new InstantCommand(() -> RobotContainer.scoreAssist.updateManually(target));
  }

  public static Command exectuteDrive(Command defaultCmd) {
    return Commands.either(
            new DriveToPose(
                () -> RobotContainer.scoreAssist.getCurrentNodeTarget().getPose(),
                RobotContainer.driveSubsystem),
            defaultCmd,
            () -> RobotContainer.scoreAssist.readyToAutoDrive())
        .repeatedly();
  }

  public static Command exectuteDriveUntilAtTarget() {
    return new DriveToPose(
            () -> RobotContainer.scoreAssist.getCurrentNodeTarget().getPose(),
            RobotContainer.driveSubsystem)
        .repeatedly()
        .until(() -> (RobotContainer.scoreAssist.readyToAlignSS()));
  }

  public static Command executeSS() {
    return new SetSuperstructureDuringScoreAssist(
        RobotContainer.scoreAssist,
        () -> RobotContainer.scoreAssist.getCurrentLevelTarget(),
        RobotContainer.elevator,
        RobotContainer.shoulder,
        RobotContainer.pivot);
  }
}
