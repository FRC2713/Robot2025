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
import frc.robot.commands.superstructure.SuperStructure;
import frc.robot.subsystems.drive.Drivetrain;
import frc.robot.util.RHRUtil;

public class RiverRage {
  /**
   * @param factory
   * @param driveSubsystem
   * @return
   */
  public static AutoRoutine getRoutine(AutoFactory factory, Drivetrain driveSubsystem) {
    AutoRoutine routine = factory.newRoutine("River Rage");

    // Load the routine's trajectories
    // StartRoReefE starts square to the starting line, Start2RoReefE starts pre-aligned to the EF
    // reef face
    AutoTrajectory startToReefGTraj = routine.trajectory("CenterPineTree");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                new InstantCommand(() -> System.out.println("CenterPineTree started")),
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
                SuperStructure.UNFOLD.get(),
                SuperStructure.L4.get(),
                startToReefGTraj.cmd(),
                Commands.print("Shoulder in position & trajectory started")));

    return routine;
  }
}
