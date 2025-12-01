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
import frc.robot.SetpointConstants;
import frc.robot.commands.ArmCmds;
import frc.robot.commands.ElevatorCmds;
import frc.robot.commands.superstructure.SuperStructure;
import frc.robot.subsystems.drive.Drivetrain;
import frc.robot.util.RHRUtil;

public class CenterAutoRebuildFlipped {
  /**
   * @param factory
   * @param driveSubsystem
   * @return
   */
  public static AutoRoutine getRoutine(AutoFactory factory, Drivetrain driveSubsystem) {
    AutoRoutine routine = factory.newRoutine("Rebuild Center Auto");

    // Load the routine's trajectories
    // StartRoReefE starts square to the starting line, Start2RoReefE starts pre-aligned to the EF
    // reef face
    AutoTrajectory startToReefGTraj =
        RHRUtil.flipHorizontal(routine.trajectory("RebuildCenter"), routine);
    AutoTrajectory reefBackupTraj =
        RHRUtil.flipHorizontal(routine.trajectory("RebuildBackup"), routine);
    AutoTrajectory reefAlignCenter =
        RHRUtil.flipHorizontal(routine.trajectory("RebuildCenterReef"), routine);
    routine
        .active()
        .onTrue(
            Commands.sequence(
                new InstantCommand(() -> System.out.println("RebuildCenter started")),
                RHRUtil.resetRotationIfReal(startToReefGTraj.getInitialPose().get()),
                // If pose estimation is really off, reset based on the trajectory
                new InstantCommand(
                    () -> {
                      if (RobotContainer.driveSubsystem.getPose().getTranslation().getX() == 0
                          || RobotContainer.driveSubsystem
                                  .getPose()
                                  .getTranslation()
                                  .getDistance(
                                      startToReefGTraj.getInitialPose().get().getTranslation())
                              > 1) {
                        System.out.println("Hard reset odom");
                        RobotContainer.driveSubsystem.resetOdometry(
                            startToReefGTraj.getInitialPose().get());
                      }
                    }),
                Commands.parallel(SuperStructure.L4_PREP.get(), startToReefGTraj.cmd()),
                Commands.print("Shoulder in position & trajectory started")));

    var command =
        Commands.sequence(
            ElevatorCmds.setHeight(24),
            Commands.parallel(
                ArmCmds.armSetAngleAndWait(SetpointConstants.Arm.L4_ANGLE_DEG_SCORE),
                ArmCmds.handSetVoltage(2)));
    // When at the reef, score
    startToReefGTraj
        .done()
        .onTrue(
            Commands.sequence(
                // 1) Finish off trajectory with score assist, which also moves the SS and scores
                // ScoreAssistCmds.executeCoralScoreInAuto(ScoreLocations.G_FOUR),
                command,
                // 2) Wait to make sure coral is outtathere
                Commands.waitSeconds(
                    0.6), // Value can ba changed if coral is missing or robot is stalling
                reefAlignCenter.cmd(),
                // Algae
                SuperStructure.ALGAE_GRAB_L2.get(),
                Commands.waitSeconds(
                    2), // Can be set to zero if the algae is being picked up in time
                reefBackupTraj.cmd()));

    return routine;
  }
}
