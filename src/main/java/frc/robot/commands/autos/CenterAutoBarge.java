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
    AutoTrajectory centreBargeOne = routine.trajectory("CentreBarge1");
    AutoTrajectory centreBargeTwo = routine.trajectory("CentreBarge2");
    AutoTrajectory centreBargeThree = routine.trajectory("CentreBarge3");

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
                new InstantCommand(() -> driveSubsystem.stop()),
                // 1) Finish off trajectory with score assist, which also moves the SS and scores
                ScoreAssistCmds.executeCoralScoreInAuto(ScoreLocations.G_FOUR),
                new InstantCommand(() -> driveSubsystem.stop()),

                // 3) Begin driving to Algae Setpoint
                centreBargeOne.cmd()));

    centreBargeOne
        .done()
        .onTrue(
            Commands.sequence(
                new InstantCommand(() -> driveSubsystem.stop()),
                SuperStructure.ALGAE_SS_L2.get(),
                centreBargeTwo.cmd()));

    centreBargeTwo
        .done()
        .onTrue(
            Commands.sequence(
                EndEffector.ALGAE_GRAB.get(),
                EndEffector.WAIT_UNTIL_ALGAE.get(),
                EndEffector.ALGAE_HOLD.get(),
                centreBargeThree.cmd()));
    centreBargeThree
        .done()
        .onTrue(
            Commands.sequence(
                SuperStructure.BARGE_PREP_FORWARDS.get(),
                Commands.waitSeconds(0.5),
                EndEffector.BARGE_SCORE.get(),
                Commands.waitSeconds(0.5),
                SuperStructure.STARTING_CONF.get()));

    return routine;
  }
}
