package frc.robot.commands.scoreassist;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.DriveCmds;
import frc.robot.scoreassist.ReefAlign;
import frc.robot.scoreassist.ScoreAssist.ScoreDrivingMode;

public class PathFindToPoseWithOverride extends SequentialCommandGroup {

  public PathFindToPoseWithOverride() {
    Command setToPathFinding =
        Commands.runOnce(() -> RobotContainer.scoreAssist.mode = ScoreDrivingMode.PATH);
    Command pathFinding =
        new PathfindToPose(
                RobotContainer.driveSubsystem,
                () -> RobotContainer.scoreAssist.getCurrentNodeTarget().getPathScorePose())
            .until(
                () ->
                    RobotContainer.scoreAssist.shouldManuallyOverridePath()
                        || !RobotContainer.scoreAssist.shouldUsePath());

    Command setToOverride =
        Commands.runOnce(() -> RobotContainer.scoreAssist.mode = ScoreDrivingMode.PATH_OVERRIDEN);
    Command overriding =
        DriveCmds.joystickDriveAtAngle(
                RobotContainer.driveSubsystem,
                () -> -RobotContainer.driverControls.getLeftY(),
                () -> -RobotContainer.driverControls.getLeftX(),
                () ->
                    ReefAlign.getInstance()
                        .inZone()
                        .orElse(RobotContainer.driveSubsystem.getRotation()))
            .until(() -> !RobotContainer.scoreAssist.shouldUsePath());
    this.addCommands(setToPathFinding, pathFinding, setToOverride, overriding);
  }
}
