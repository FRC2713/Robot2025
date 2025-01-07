package frc.robot.commands;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drivetrain;

public class FullAutoRoutines {

  private static Command resetToInitialPose(
      Drivetrain drive, AutoRoutine routine, AutoTrajectory initialTrajectory) {
    return Commands.runOnce(
        () ->
            initialTrajectory
                .getInitialPose()
                .ifPresentOrElse(pose -> drive.resetOdometry(pose), routine::kill),
        drive);
  }

  public static Command simplePathAuto(Drivetrain drive, AutoFactory factory) {
    AutoRoutine routine = factory.newRoutine("simplePathAuto");
    routine.active().onTrue(Commands.print("Started Routine: simplePathAuto"));

    // Follows deploy/choreo/SimplePath.traj
    AutoTrajectory simplePath = routine.trajectory("SimplePath");

    routine
        .active()
        .onTrue(
            Commands.sequence(resetToInitialPose(drive, routine, simplePath), simplePath.cmd()));

    return routine.cmd();
  }

  // see the section on Choreo's website about event markers and branching autos:
  // https://choreo.autos/choreolib/auto-factory/ under `Using AutoRoutine`

}
