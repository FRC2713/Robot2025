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
import frc.robot.commands.IntakeCmds;
import frc.robot.commands.superstructure.SuperStructure;
import frc.robot.subsystems.constants.IntakeConstants;
import frc.robot.subsystems.drive.Drivetrain;
import frc.robot.util.RHRUtil;

public class CenterAutoRebuild {
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
    AutoTrajectory startToReefGTraj = routine.trajectory("RebuildCenter");
    AutoTrajectory reefAlignCenter = routine.trajectory("RebuildCenterReef");
    AutoTrajectory reefBackupTraj = routine.trajectory("RebuildBackup");

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
                Commands.sequence(
                    Commands.sequence(SuperStructure.UNFOLD.get(), SuperStructure.L4_FLIPPED.get()),
                    startToReefGTraj.cmd()),
                Commands.print("Shoulder in position & trajectory started")));

    // When at the reef, score
    startToReefGTraj
        .done()
        .onTrue(
            Commands.sequence(
                // 1) Finish off trajectory with score assist, which also moves the SS and scores
                // ScoreAssistCmds.executeCoralScoreInAuto(ScoreLocations.G_FOUR),
                SuperStructure.L4_SCORE_FLIPPED.get(),
                // 2) Wait to make sure coral is outtathere
                Commands.waitSeconds(
                    0.6), // Value can ba changed if coral is missing or robot is stalling
                Commands.parallel(
                    SuperStructure.ALGAE_GRAB_L2_FLIPPED.get(), reefAlignCenter.cmd()),
                // Algae
                Commands.waitSeconds(
                    2), // Can be set to zero if the algae is being picked up in time
                Commands.parallel(
                    reefBackupTraj.cmd(),
                    Commands.sequence(
                        Commands.waitSeconds(0.5), SuperStructure.ALGAE_SCORE_FLIPPED.get())),
                ArmCmds.handSetVoltage(8),
                Commands.waitSeconds(1),

                // Reset for teleop
                IntakeCmds.setAngleAndWait(IntakeConstants.kIPMaxAngle - 5),
                ElevatorCmds.setHeightAndWait(SetpointConstants.Elevator.ELEVATOR_HANDOFF_HEIGHT),
                IntakeCmds.setAngle(SetpointConstants.Intake.INTAKE_HANDOFF_ANGLE),
                ArmCmds.armSetAngle(-90)));

    return routine;
  }
}
