// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.SSConstants;
import frc.robot.commands.SuperStructure;
import frc.robot.commands.scoreassist.ScoreAssistCmds;
import frc.robot.subsystems.drive.Drivetrain;
import frc.robot.util.RHRUtil;
import frc.robot.util.ScoreLoc;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ScoreLotsOfCoral {
  /**
   * @param factory
   * @param driveSubsystem
   * @return
   */
  public static AutoRoutine getRoutine(AutoFactory factory, Drivetrain driveSubsystem) {
    AutoRoutine routine = factory.newRoutine("Score Lots of Coral");

    // Load the routine's trajectories
    // StartRoReefE starts square to the starting line, Start2RoReefE starts pre-aligned to the EF
    // reef face
    AutoTrajectory startToReefETraj = routine.trajectory("StartToReefE");
    AutoTrajectory reefEToSource = routine.trajectory("ReefEToSource");
    AutoTrajectory sourceToReefC = routine.trajectory("SourceToReefC");
    AutoTrajectory reefCToSource = routine.trajectory("ReefCToSource");
    AutoTrajectory sourceToReefD = routine.trajectory("SourceToReefD");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                new InstantCommand(() -> System.out.println("Score Lots of Coral started")),
                RHRUtil.resetRotationIfReal(startToReefETraj.getInitialPose().get()),
                new InstantCommand(
                    () -> {
                      if (RobotContainer.driveSubsystem.getPose().getTranslation().getX() == 0
                          || RobotContainer.driveSubsystem
                                  .getPose()
                                  .getTranslation()
                                  .getDistance(
                                      startToReefETraj.getInitialPose().get().getTranslation())
                              > 1) {
                        System.out.println("Hard reset odom");
                        RobotContainer.driveSubsystem.resetOdometry(
                            startToReefETraj.getInitialPose().get());
                      }
                    }),
                Commands.parallel(SuperStructure.L4_PREP.getCommand(), startToReefETraj.cmd()),
                Commands.print("Shoulder in position & trajectory started")));

    // When at the reef, score and go to source
    startToReefETraj
        .done()
        .onTrue(
            Commands.sequence(
                // 1) Finish off trajectory with score assist, which also moves the SS
                ScoreAssistCmds.exectuteCoralScoreInAuto(ScoreLoc.E_FOUR),
                // 2) Score Coral
                // Commands.waitSeconds(SSConstants.Auto.L4_SCORE_DELAY.getAsDouble()),
                // Commands.print("Scoring Coral"),
                // SuperStructure.CORAL_SCORE.getCommand(),
                // TODO: Tune down
                Commands.waitSeconds(SSConstants.Auto.L4_POST_SCORE_DELAY.getAsDouble()),
                // 3) Begin driving to source
                Commands.parallel(
                    SuperStructure.SOURCE_CORAL_INTAKE.getCommand(), reefEToSource.cmd())));

    // When the trajectory is done, intake; then go to reef B
    reefEToSource
        .done()
        .onTrue(
            Commands.sequence(
                new InstantCommand(() -> driveSubsystem.stop()),
                Commands.race(
                    Commands.parallel(
                        SuperStructure.SOURCE_CORAL_INTAKE.getCommand(),
                        new WaitUntilCommand(() -> RobotContainer.rollers.hasCoral())),
                    Commands.waitSeconds(2.0)),
                sourceToReefC.cmd()));

    // Prep elevator along the way
    // sourceToReefB.atTime("PrepElevator").onTrue(SuperStructure.L4_PREP.getCommand());
    // Once at reef B, score and go to source
    sourceToReefC
        .done()
        .onTrue(
            Commands.sequence(
                // 1) Finish off trajectory with score assist, which also moves the SS
                ScoreAssistCmds.exectuteCoralScoreInAuto(ScoreLoc.C_FOUR),
                // TODO: Tune down
                Commands.waitSeconds(SSConstants.Auto.L4_POST_SCORE_DELAY.getAsDouble()),
                // 3) Begin driving to source
                Commands.parallel(
                    SuperStructure.SOURCE_CORAL_INTAKE.getCommand(), reefCToSource.cmd())));

    reefCToSource
        .done()
        .onTrue(
            Commands.sequence(
                new InstantCommand(() -> driveSubsystem.stop()),
                Commands.race(
                    Commands.parallel(
                        SuperStructure.SOURCE_CORAL_INTAKE.getCommand(),
                        new WaitUntilCommand(() -> RobotContainer.rollers.hasCoral())),
                    Commands.waitSeconds(2.0)),
                sourceToReefD.cmd()));

    sourceToReefD
        .done()
        .onTrue(
            Commands.sequence(
                // 1) Finish off trajectory with score assist, which also moves the SS
                ScoreAssistCmds.exectuteCoralScoreInAuto(ScoreLoc.D_FOUR),
                // TODO: Tune down
                Commands.waitSeconds(SSConstants.Auto.L4_POST_SCORE_DELAY.getAsDouble()),
                // 3) Begin driving to source
                Commands.parallel(
                    SuperStructure.SOURCE_CORAL_INTAKE.getCommand(), reefCToSource.cmd())));
    // Commands.sequence(
    //     // 1) Finish off trajectory with score assist, in parallel move SS to L4
    //     new ParallelDeadlineGroup(
    //         Commands.waitSeconds(2),
    //         Commands.parallel(
    //             Commands.sequence(
    //                 new InstantCommand(
    //                     () -> ScoreAssistOld.getInstance().setReefTrackerLoc(ScoreLoc.C_FOUR)),
    //                 Commands.print("2Waiting for ScoreAssistOld"),
    //                 ScoreAssistOld.getInstance()
    //                     .goReefTracker(driveSubsystem)
    //                     .withDeadline(ScoreAssistOld.getInstance().waitUntilFinished(1.0)),
    //                 Commands.print("2ScoreAssistOld Done")))),
    //     new InstantCommand(() -> driveSubsystem.stop()),
    //     Commands.print("2driveSubsystem stop thing Done"),
    //     Commands.print("2L4"),
    //     SuperStructure.L4.getCommand().withDeadline(Commands.waitSeconds(2)),
    //     Commands.print("2L4 Done"),
    //     // 2) Score Coral
    //     Commands.waitSeconds(SSConstants.Auto.L4_SCORE_DELAY.getAsDouble()),
    //     Commands.print("2Scoring Coral"),
    //     SuperStructure.CORAL_SCORE.getCommand(),
    //     Commands.waitSeconds(SSConstants.Auto.L4_POST_SCORE_DELAY.getAsDouble()),
    //     // 3) Begin driving to source
    //     Commands.parallel(
    //         SuperStructure.SOURCE_CORAL_INTAKE.getCommand(), reefCToSource.cmd())));
    // Commands.sequence(
    //     // 1) Finish off trajectory with score assist, in parallel move SS to L4
    //     Commands.parallel(
    //         Commands.sequence(
    //             new InstantCommand(
    //                 () -> ScoreAssistOld.getInstance().setReefTrackerLoc(ScoreLoc.C_FOUR)),
    //             ScoreAssistOld.getInstance()
    //                 .goReefTracker(driveSubsystem)
    //                 .withDeadline(ScoreAssistOld.getInstance().waitUntilFinished(1.0)),
    //             new InstantCommand(() -> driveSubsystem.stop()))),
    //     SuperStructure.L4.getCommand(),
    //     // 2) Score Coral
    //     SuperStructure.CORAL_SCORE.getCommand(),
    //     // 3) Begin driving to source
    //     Commands.parallel(
    //         SuperStructure.SOURCE_CORAL_INTAKE.getCommand(), reefCToSource.cmd())));

    return routine;
  }
}
