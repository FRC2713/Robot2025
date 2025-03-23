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
import frc.robot.commands.superstructure.SuperStructure;
import frc.robot.subsystems.drive.Drivetrain;
import frc.robot.util.RHRUtil;
import frc.robot.util.ScoreLoc.ScoreLocations;

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
    AutoTrajectory reefDToSource = routine.trajectory("ReefDToSource");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                new InstantCommand(() -> System.out.println("Score Lots of Coral started")),
                RHRUtil.resetRotationIfReal(startToReefETraj.getInitialPose().get()),
                // If pose estimation is really off, reset based on the trajectory
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
                Commands.parallel(SuperStructure.L4_PREP.get(), startToReefETraj.cmd()),
                Commands.print("Shoulder in position & trajectory started")));

    // When at the reef, score and go to source
    startToReefETraj
        .done()
        .onTrue(
            Commands.sequence(
                // 1) Finish off trajectory with score assist, which also moves the SS and scores
                ScoreAssistCmds.exectuteCoralScoreInAuto(ScoreLocations.E_FOUR),
                // TODO: Tune down
                Commands.waitSeconds(SetpointConstants.Auto.L4_POST_SCORE_DELAY.getAsDouble()),
                // 3) Begin driving to source
                Commands.parallel(SuperStructure.SOURCE_CORAL_INTAKE.get(), reefEToSource.cmd())));

    // When the trajectory is done, intake; then go to reef B
    reefEToSource
        .done()
        .onTrue(
            Commands.sequence(
                new InstantCommand(() -> driveSubsystem.stop()),
                Commands.race(
                    Commands.parallel(
                        SuperStructure.SOURCE_CORAL_INTAKE.get(),
                        new WaitUntilCommand(() -> RobotContainer.rollers.hasCoral())),
                    Commands.waitSeconds(0.1)),
                sourceToReefC.cmd()));

    // Prep elevator along the way
    // sourceToReefB.atTime("PrepElevator").onTrue(SuperStructure.L4_PREP);
    // Once at reef B, score and go to source
    sourceToReefC
        .done()
        .onTrue(
            Commands.sequence(
                // 1) Finish off trajectory with score assist, which also moves the SS and runs the
                // rollers
                ScoreAssistCmds.exectuteCoralScoreInAuto(ScoreLocations.C_FOUR),
                // TODO: Tune down
                Commands.waitSeconds(SetpointConstants.Auto.L4_POST_SCORE_DELAY.getAsDouble()),
                // 3) Move SS to intake configuration and begin driving to source
                Commands.parallel(SuperStructure.SOURCE_CORAL_INTAKE.get(), reefCToSource.cmd())));

    reefCToSource
        .done()
        .onTrue(
            Commands.sequence(
                new InstantCommand(() -> driveSubsystem.stop()),
                Commands.race(
                    Commands.parallel(
                        SuperStructure.SOURCE_CORAL_INTAKE.get(),
                        new WaitUntilCommand(() -> RobotContainer.rollers.hasCoral())),
                    Commands.waitSeconds(0.1)),
                sourceToReefD.cmd()));

    sourceToReefD
        .done()
        .onTrue(
            Commands.sequence(
                // 1) Finish off trajectory with score assist, which also moves the SS
                ScoreAssistCmds.exectuteCoralScoreInAuto(ScoreLocations.D_FOUR),
                // TODO: Tune down
                Commands.waitSeconds(SetpointConstants.Auto.L4_POST_SCORE_DELAY.getAsDouble()),
                // 3) Begin driving to source
                Commands.parallel(SuperStructure.SOURCE_CORAL_INTAKE.get(), reefDToSource.cmd())));

    return routine;
  }
}
