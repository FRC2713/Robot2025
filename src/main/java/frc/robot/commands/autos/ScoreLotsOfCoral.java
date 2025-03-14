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
import frc.robot.commands.ScoreAssist;
import frc.robot.commands.SuperStructure;
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

    routine
        .active()
        .onTrue(
            Commands.sequence(
                new InstantCommand(() -> System.out.println("Score Lots of Coral started")),
                RHRUtil.resetRotationIfReal(startToReefETraj.getInitialPose().get()),
                Commands.parallel(SuperStructure.L4_PREP.getCommand(), startToReefETraj.cmd()),
                Commands.print("Shoulder in position & trajectory started")));

    // When at the reef, score and go to source
    startToReefETraj
        .done()
        .onTrue(
            Commands.sequence(
                // 1) Finish off trajectory with score assist, in parallel move SS to L4
                Commands.parallel(
                    Commands.sequence(
                        new InstantCommand(
                            () -> ScoreAssist.getInstance().setReefTrackerLoc(ScoreLoc.E_FOUR)),
                        ScoreAssist.getInstance()
                            .goReefTracker(driveSubsystem)
                            .withDeadline(ScoreAssist.getInstance().waitUntilFinished(1.0)),
                        new InstantCommand(() -> driveSubsystem.stop())),
                    SuperStructure.L4.getCommand()),
                // 2) Score Coral
                ScoreAssist.getInstance().waitUntilFinished(1.6),
                Commands.waitSeconds(SSConstants.Auto.L4_SCORE_DELAY.getAsDouble()),
                SuperStructure.CORAL_SCORE.getCommand(),
                Commands.waitSeconds(SSConstants.Auto.L4_POST_SCORE_DELAY.getAsDouble()),
                // 3) Begin driving to source
                Commands.parallel(
                    SuperStructure.SOURCE_CORAL_INTAKE.getCommand(), reefEToSource.cmd())));

    // When the trajectory is done, intake; then go to reef B
    reefEToSource
        .done()
        .onTrue(
            Commands.sequence(
                SuperStructure.SOURCE_CORAL_INTAKE.getCommand(),
                Commands.race(
                    new WaitUntilCommand(() -> RobotContainer.rollers.hasCoral()),
                    Commands.waitSeconds(1.0)),
                sourceToReefC.cmd()));

    // Prep elevator along the way
    // sourceToReefB.atTime("PrepElevator").onTrue(SuperStructure.L4_PREP.getCommand());
    // Once at reef B, score and go to source
    sourceToReefC
        .done()
        .onTrue(
            Commands.sequence(
                // 1) Finish off trajectory with score assist, in parallel move SS to L4
                Commands.parallel(
                    Commands.sequence(
                        new InstantCommand(
                            () -> ScoreAssist.getInstance().setReefTrackerLoc(ScoreLoc.C_FOUR)),
                        ScoreAssist.getInstance()
                            .goReefTracker(driveSubsystem)
                            .withDeadline(ScoreAssist.getInstance().waitUntilFinished(1.0)),
                        new InstantCommand(() -> driveSubsystem.stop())),
                    SuperStructure.L4.getCommand()),
                // 2) Score Coral
                SuperStructure.CORAL_SCORE.getCommand(),
                // 3) Begin driving to source
                Commands.parallel(
                    SuperStructure.SOURCE_CORAL_INTAKE.getCommand(), reefCToSource.cmd())));

    return routine;
  }
}
