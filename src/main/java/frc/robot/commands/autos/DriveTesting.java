package frc.robot.commands.autos;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.drive.Drivetrain;

public class DriveTesting {

  public static AutoRoutine getRoutine(AutoFactory m_factory, Drivetrain driveSubsystem) {
    AutoRoutine routine = m_factory.newRoutine("DriveTesting");
    AutoTrajectory driveTesting = routine.trajectory("DriveTesting");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                new InstantCommand(() -> System.out.println("Drive Testing 3m")),
                driveTesting.resetOdometry(),
                driveTesting.cmd()));
    return routine;
  }

  public static AutoRoutine getSLAMDunkRoutine(AutoFactory m_factory, Drivetrain driveSubsystem) {
    AutoRoutine routine = m_factory.newRoutine("SLAMDunk");
    AutoTrajectory driveTesting = routine.trajectory("SLAMDunkTest");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                new InstantCommand(() -> System.out.println("Drive Testing 1ft")),
                driveTesting.resetOdometry(),
                driveTesting.cmd()));
    return routine;
  }
}
