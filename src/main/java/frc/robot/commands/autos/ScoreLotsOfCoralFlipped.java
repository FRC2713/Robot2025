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
import frc.robot.SetpointConstants;
import frc.robot.commands.scoreassist.ScoreAssistCmds;
import frc.robot.commands.superstructure.EndEffector;
import frc.robot.commands.superstructure.SuperStructure;
import frc.robot.subsystems.drive.Drivetrain;
import frc.robot.util.RHRUtil;
import frc.robot.util.ScoreLoc.ScoreLocations;

// TODO: make sure this actually matches ScoreLotsOfCoral with all the refactoring and tuning
public class ScoreLotsOfCoralFlipped {
  /**
   * @param factory
   * @param driveSubsystem
   * @return
   */
  public static AutoRoutine getRoutine(AutoFactory factory, Drivetrain driveSubsystem) {
    AutoRoutine routine = factory.newRoutine("Score Lots of Coral Flipped");

    // Load the routine's trajectories
    AutoTrajectory startToReefJTraj = routine.trajectory("StartToReefJ");
    AutoTrajectory reefJToSource = routine.trajectory("ReefJToSource");
    AutoTrajectory sourceToReefL = routine.trajectory("SourceToReefL");
    AutoTrajectory reefLToSource = routine.trajectory("ReefLToSource");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                new InstantCommand(() -> System.out.println("Score Lots of Coral started")),
                RHRUtil.resetRotationIfReal(startToReefJTraj.getInitialPose().get()),
                Commands.parallel(SuperStructure.L4_PREP.get(), startToReefJTraj.cmd()),
                Commands.print("Shoulder in position & trajectory started")));

    // When at the reef, score and go to source
    startToReefJTraj
        .done()
        .onTrue(
            Commands.sequence(
                // 1) Finish off trajectory with score assist, in parallel move SS to L4 and when
                // ready runs the rollers
                ScoreAssistCmds.exectuteCoralScoreInAuto(ScoreLocations.J_FOUR),
                Commands.waitSeconds(SetpointConstants.Auto.L4_POST_SCORE_DELAY.getAsDouble()),
                // 2) Begin driving to source
                Commands.parallel(SuperStructure.SOURCE_CORAL_INTAKE.get(), reefJToSource.cmd())));

    // When the trajectory is done, intake; then go to reef B
    reefJToSource
        .done()
        .onTrue(
            Commands.sequence(
                SuperStructure.SOURCE_CORAL_INTAKE.get(),
                Commands.race(
                    new WaitUntilCommand(() -> RobotContainer.rollers.hasCoral()),
                    Commands.waitSeconds(1.0)),
                sourceToReefL.cmd()));

    // Prep elevator along the way
    // sourceToReefB.atTime("PrepElevator").onTrue(SuperStructure.L4_PREP);
    // Once at reef B, score and go to source
    sourceToReefL
        .done()
        .onTrue(
            Commands.sequence(
                // 1) Finish off trajectory with score assist, in parallel move SS to L4
                ScoreAssistCmds.exectuteCoralScoreInAuto(ScoreLocations.L_FOUR),
                // 2) Score Coral
                EndEffector.CORAL_SCORE.get(),
                // 3) Begin driving to source
                Commands.parallel(SuperStructure.SOURCE_CORAL_INTAKE.get(), reefLToSource.cmd())));

    return routine;
  }
}
