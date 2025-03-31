// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.AlgaeClawCmds;
import frc.robot.commands.scoreassist.ScoreAssistCmds;
import frc.robot.commands.superstructure.EndEffector;
import frc.robot.commands.superstructure.SuperStructure;
import frc.robot.subsystems.drive.Drivetrain;
import frc.robot.util.RHRUtil;
import frc.robot.util.ScoreLoc.ScoreLocations;

public class CenterAutoBarge {
  /**
   * @param factory
   * @param driveSubsystem
   * @return
   */
  public static AutoRoutine getRoutine(AutoFactory factory, Drivetrain driveSubsystem) {
    AutoRoutine routine = factory.newRoutine("Center Auto");

    // Load the routine's trajectories
    // StartRoReefE starts square to the starting line, Start2RoReefE starts pre-aligned to the EF
    // reef face
    AutoTrajectory centreAlgae = routine.trajectory("CentreAlgae");
    AutoTrajectory centreBarge = routine.trajectory("CentreBarge");
    // AutoTrajectory sourceToReefC = routine.trajectory("SourceToReefC");
    // AutoTrajectory reefCToSource = routine.trajectory("ReefCToSource");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                new InstantCommand(() -> System.out.println("CenterAutoBarge started")),
                RHRUtil.resetRotationIfReal(centreAlgae.getInitialPose().get()),
                // If pose estimation is really off, reset based on the trajectory
                new InstantCommand(
                    () -> {
                      if (RobotContainer.driveSubsystem.getPose().getTranslation().getX() == 0
                          || RobotContainer.driveSubsystem
                                  .getPose()
                                  .getTranslation()
                                  .getDistance(centreAlgae.getInitialPose().get().getTranslation())
                              > 1) {
                        System.out.println("Hard reset odom");
                        RobotContainer.driveSubsystem.resetOdometry(
                            centreAlgae.getInitialPose().get());
                      }
                    }),
                Commands.parallel(SuperStructure.L4_PREP.get(), centreAlgae.cmd()),
                Commands.print("Shoulder in position & trajectory started")));

    // When at the reef, score
    centreAlgae
        .done()
        .onTrue(
            Commands.sequence(
                // 1) Finish off trajectory with score assist, which also moves the SS and scores
                ScoreAssistCmds.executeCoralScoreInAuto(ScoreLocations.G_FOUR),

                // 3) Score coral
                ScoreAssistCmds.executeAlgaeGrabInAuto(ScoreLocations.ALGAE_GH)
                    .withDeadline(Commands.waitSeconds(3)),
                // 2) Wait to make sure we got algae
                AlgaeClawCmds.waitUntilAlgae(1),

                // 3) Begin driving to barge
                Commands.parallel(
                    centreBarge.cmd(), SuperStructure.STARTING_CONF_WITH_ALGAE.get())));

    centreBarge
        .done()
        .onTrue(
            Commands.sequence(
                SuperStructure.BARGE_PREP_FORWARDS.get(), EndEffector.BARGE_SCORE.get()));

    return routine;
  }
}
