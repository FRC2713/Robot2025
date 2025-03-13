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
import frc.robot.SSConstants;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ScoreAssist;
import frc.robot.commands.SuperStructure;
import frc.robot.subsystems.drive.Drivetrain;
import frc.robot.util.RHRUtil;

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
    AutoTrajectory startToReefDTraj = routine.trajectory("Start2ToReefD");
    AutoTrajectory reefDToSource = routine.trajectory("ReefDToSource");
    AutoTrajectory sourceToReefB = routine.trajectory("SourceToReefB");
    AutoTrajectory reefBToSource = routine.trajectory("ReefBToSource");
    AutoTrajectory sourceToReefA = routine.trajectory("SourceToReefA");
    AutoTrajectory reefAToSource = routine.trajectory("ReefAToSource");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                new InstantCommand(() -> System.out.println("Score Lots of Coral started")),
                RHRUtil.resetRotationIfReal(startToReefDTraj.getInitialPose().get()),
                Commands.parallel(SuperStructure.L4_PREP.getCommand(), startToReefDTraj.cmd())));

    // When at the reef, score and go to source
    startToReefDTraj
        .done()
        .onTrue(
            // Enable ScoreAssist
            DriveCommands.changeDefaultDriveCommand(
                    driveSubsystem,
                    ScoreAssist.getInstance().goClosest(driveSubsystem),
                    "ScoreAssist")
                .alongWith(
                    Commands.print("Waiting on Score Assist"),
                    new WaitUntilCommand(ScoreAssist.getInstance()::hasFinished),
                    Commands.print("Score Assist Done!"),
                    SuperStructure.L4.getCommand(),
                    Commands.waitSeconds(SSConstants.Auto.L4_SCORE_DELAY.getAsDouble()),
                    SuperStructure.CORAL_SCORE.getCommand(),
                    Commands.waitSeconds(SSConstants.Auto.L4_POST_SCORE_DELAY.getAsDouble()))
            // Wait Until Score Assist is Done
            //     Commands.sequence(
            //       Commands.print("Scoreassist started!"),
            // new WaitUntilCommand(
            //   () -> ScoreAssist.getInstance().error < 1),
            // Commands.print("ScoreAssit finished"),
            // SuperStructure.L4.getCommand(),
            // Commands.waitSeconds(SSConstants.Auto.L4_SCORE_DELAY.getAsDouble()),
            // SuperStructure.CORAL_SCORE.getCommand(),
            // Commands.waitSeconds(SSConstants.Auto.L4_POST_SCORE_DELAY.getAsDouble()),
            // Commands.parallel(
            //     SuperStructure.SOURCE_CORAL_INTAKE.getCommand(), reefDToSource.cmd())))
            );

    // // // When the trajectory is done, intake; then go to reef B
    // reefDToSource
    //     .done()
    //     .onTrue(
    //         Commands.sequence(
    //             SuperStructure.SOURCE_CORAL_INTAKE.getCommand()
    //             // sourceToReefB.cmd()
    //             ));

    // // // Prep elevator along the way
    // // // sourceToReefB.atTime("PrepElevator").onTrue(SuperStructure.L3_PREP.getCommand());
    // // // Once at reef B, score and go to source
    // sourceToReefB
    //     .done()
    //     .onTrue(
    //         Commands.sequence(
    //             SuperStructure.L4.getCommand(),
    //             SuperStructure.CORAL_SCORE.getCommand(),
    //             Commands.parallel(
    //                 SuperStructure.SOURCE_CORAL_INTAKE.getCommand()
    //                 // , reefBToSource.cmd()
    //                 )));

    return routine;
  }
}
