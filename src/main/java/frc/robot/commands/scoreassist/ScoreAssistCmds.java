package frc.robot.commands.scoreassist;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.DriveAtLongitude;
import frc.robot.commands.MoveSSToTarget;
import frc.robot.commands.superstructure.EndEffector;
import frc.robot.commands.superstructure.SuperStructure;
import frc.robot.scoreassist.ScoreAssist.ScoreDrivingMode;
import frc.robot.subsystems.constants.ScoreAssistConstants;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.ReefTracker;
import frc.robot.util.ScoreAssistMessage;
import frc.robot.util.ScoreLoc;
import java.util.HashMap;

public class ScoreAssistCmds {
  public static ScoreAssistMessage.GoalType contextualScore = ScoreAssistMessage.GoalType.CORAL;

  /** Execute desired score command from ReefTracker */
  public static Command executeReefTrackerScore() {
    var commands = new HashMap<ScoreAssistMessage.GoalType, Command>();
    commands.put(ScoreAssistMessage.GoalType.CORAL, executeCoralScore());
    commands.put(ScoreAssistMessage.GoalType.ALGAE, executeAlgaeGrab());
    commands.put(ScoreAssistMessage.GoalType.BARGE, executeBargeScore());
    commands.put(ScoreAssistMessage.GoalType.PROCESSOR, executeProcessorScore());
    return Commands.select(commands, ReefTracker.getInstance()::getGoalTypeOrCoral);
  }

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

  // Gets the desired manual score type
  public static Command contextualManualScore() {
    return Commands.select(
        new HashMap<ScoreAssistMessage.GoalType, Command>() {
          {
            put(ScoreAssistMessage.GoalType.CORAL, EndEffector.CORAL_SCORE.get());
            put(ScoreAssistMessage.GoalType.BARGE, EndEffector.BARGE_SCORE.get());
            put(ScoreAssistMessage.GoalType.PROCESSOR, EndEffector.PROCESSOR_SCORE.get());
          }
        },
        () -> contextualScore);
  }

  /** This is assist for barge scoring */
  public static Command executeBargeScore() {
    return Commands.parallel(
            Commands.runOnce(() -> contextualScore = ScoreAssistMessage.GoalType.BARGE),
            Commands.runOnce(() -> RobotContainer.scoreAssist.mode = ScoreDrivingMode.BARGE),
            Commands.either(
                SuperStructure.BARGE_PREP_BACKWARDS.get(),
                SuperStructure.BARGE_PREP_FORWARDS.get(),
                () ->
                    AllianceFlipUtil.shouldFlip()
                        ? !DriveAtLongitude.doBackwards(ScoreAssistConstants.bargeAlignmentX)
                        : DriveAtLongitude.doBackwards(ScoreAssistConstants.bargeAlignmentX)),
            new DriveAtLongitude(
                () -> AllianceFlipUtil.apply(ScoreAssistConstants.bargeAlignmentX),
                RobotContainer.driveSubsystem))
        .finallyDo(() -> RobotContainer.scoreAssist.mode = ScoreDrivingMode.INACTIVE);
  }

  /** This drives to target, moves the SS when ready, and runs the rollers when ready */
  public static Command executeCoralScore() {
    return Commands.sequence(
            Commands.runOnce(() -> contextualScore = ScoreAssistMessage.GoalType.CORAL),
            Commands.parallel(
                start(), // 1) activate score assist
                Commands.either(
                    Commands.sequence(
                        // Commands.deadline(
                        executePath(),
                        // executePrep()
                        //     .beforeStarting(
                        //         Commands.waitSeconds(
                        //             1))), // 2a) path-find close to target (with manual
                        // override)
                        executeDrive() // 2b) drive to target
                        ),
                    Commands.parallel(executeDrive(), executePrep()),
                    RobotContainer.scoreAssist::isAtPathTargetPose)), // 3) execute prep
            stop(),
            executeSS())
        .finallyDo(() -> RobotContainer.scoreAssist.mode = ScoreDrivingMode.INACTIVE);
  }

  /** This drives to target, moves the SS when ready, and grabs algae when ready */
  // private static Command executeAlgaeGrabPathing() {
  //   return Commands.sequence(
  //           Commands.runOnce(() -> contextualScore = ScoreAssistMessage.GoalType.CORAL),
  //           start(), // 1) activate score assist
  //           Commands.parallel(
  //               executePrep(), executePath()), // 2a) path-find close to target (with manual
  //           // override)
  //           executeDrive(), // 2b) drive to target
  //           stop(),
  //           executeSS())
  //       .finallyDo(() -> RobotContainer.scoreAssist.mode = ScoreDrivingMode.INACTIVE);
  // }

  /** This drives to target, moves the SS when ready, and grabs algae when ready */
  private static Command executeAlgaeGrab() {
    return Commands.sequence(
            Commands.runOnce(() -> contextualScore = ScoreAssistMessage.GoalType.CORAL),
            start(), // 1) activate score assist
            Commands.parallel(
                executePrep(),
                executeDriveToPathPose()), // 2a) path-find close to target (with manual
            // override)
            executeDrive(), // 2b) drive to target
            stop(),
            executeSS())
        .finallyDo(() -> RobotContainer.scoreAssist.mode = ScoreDrivingMode.INACTIVE);
  }

  /** This drives to target, moves the SS when ready, and scores in the processor when ready */
  private static Command executeProcessorScore() {
    return Commands.parallel(
        Commands.runOnce(() -> contextualScore = ScoreAssistMessage.GoalType.PROCESSOR),
        start(),
        SuperStructure.PROCESSOR_PREP.get()
        // Commands.sequence(
        //     new DriveToPose(
        //         () -> AllianceFlipUtil.apply(ScoreAssistConstants.processorPose),
        //         RobotContainer.driveSubsystem),
        //     EndEffector.PROCESSOR_SCORE.get()
        )
    .finallyDo(() -> RobotContainer.scoreAssist.mode = ScoreDrivingMode.INACTIVE);
  }

  /** This drives to target, moves the SS when ready, and runs the rollers when ready */
  public static Command executeCoralScoreInAuto(ScoreLoc scoreLoc) {
    // Note that during autonomous, ScoreAssist does not update via NT
    return Commands.sequence(
            Commands.runOnce(() -> RobotContainer.scoreAssist.updateManually(scoreLoc)),
            Commands.parallel(
                start(), // 1) activate score assist
                executeDrive(), // 2) drive to target
                executePrep()), // 3) execute prep
            stop(),
            executeSS())
        .finallyDo(() -> RobotContainer.scoreAssist.mode = ScoreDrivingMode.INACTIVE);
  }

  /** This drives to target, moves the SS when ready, and grabs algae when ready */
  public static Command executeAlgaeGrabInAuto(ScoreLoc scoreLoc) {
    return Commands.sequence(
            Commands.runOnce(() -> RobotContainer.scoreAssist.updateManually(scoreLoc)),
            Commands.runOnce(() -> contextualScore = ScoreAssistMessage.GoalType.CORAL),
            start(), // 1) activate score assist
            Commands.parallel(
                executePrep(),
                executeDriveToPathPose()), // 2a) path-find close to target (with manual
            // override)
            executeDrive(), // 2b) drive to target
            stop(),
            executeSS())
        .finallyDo(() -> RobotContainer.scoreAssist.mode = ScoreDrivingMode.INACTIVE);
  }

  private static Command start() {
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

  /** This drives to target */
  private static Command executePath() {
    return Commands.sequence(
        Commands.runOnce(() -> RobotContainer.scoreAssist.mode = ScoreDrivingMode.PATH),
        new PathfindToPose(
            RobotContainer.driveSubsystem,
            () -> RobotContainer.scoreAssist.getCurrentNodeTarget().getPathScorePose()));
  }

  /** This drives */
  private static Command executePathWithOverride() {
    return new PathFindToPoseWithOverride();
  }
  /** This drives */
  private static Command executeDrive() {
    return Commands.sequence(
        Commands.runOnce(() -> RobotContainer.scoreAssist.mode = ScoreDrivingMode.ASSIST),
        new DriveToPose(
            () -> RobotContainer.scoreAssist.getCurrentNodeTarget().getRobotAlignmentPose(),
            RobotContainer.driveSubsystem));
  }

  /** This drives to the path pose */
  private static Command executeDriveToPathPose() {
    return Commands.sequence(
        Commands.runOnce(() -> RobotContainer.scoreAssist.mode = ScoreDrivingMode.ASSIST),
        new DriveToPose(
            () -> RobotContainer.scoreAssist.getCurrentNodeTarget().getPathScorePose(),
            RobotContainer.driveSubsystem));
  }

  /** This moves the SS when ready */
  private static Command executePrep() {
    return new MoveSSToTarget(RobotContainer.scoreAssist::getCurrentLevelTarget, true);
  }

  /** This moves the SS when ready, and runs the rollers when ready */
  private static Command executeSS() {
    return new MoveSSToTarget(RobotContainer.scoreAssist::getCurrentLevelTarget, false, true);
  }
}
