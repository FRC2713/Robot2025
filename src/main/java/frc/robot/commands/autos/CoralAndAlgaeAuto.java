package frc.robot.commands.autos;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.superstructure.EndEffector;
import frc.robot.commands.superstructure.SuperStructure;
import frc.robot.subsystems.drive.Drivetrain;

public class CoralAndAlgaeAuto {

  public static AutoRoutine getRoutine(AutoFactory m_factory, Drivetrain driveSubsystem) {
    String name = "Coral and Algae Auto";
    AutoRoutine routine = m_factory.newRoutine(name);

    // Load the routine's trajectories
    AutoTrajectory startToReefETraj = routine.trajectory("Start2ToReefE");
    AutoTrajectory reefEToProcTraj = routine.trajectory("ReefEToProcessor");
    AutoTrajectory procToSourceTraj = routine.trajectory("ProcessorToSource");
    AutoTrajectory sourceToReefC = routine.trajectory("SourceToReefC");
    AutoTrajectory reefCToProcessor = routine.trajectory("ReefCToProcessor");

    // When the routine begins, reset odometry and start the first trajectory
    routine
        .active()
        .onTrue(
            Commands.sequence(
                Commands.print(name + " started!"),
                startToReefETraj.resetOdometry(),
                startToReefETraj.cmd()));

    // Go from start to reef D, preping the elevator along the way
    startToReefETraj.atTime("PrepElevator").onTrue(SuperStructure.L3.get());

    // When at the reef, score and pick up algae; then go to the processor
    startToReefETraj
        .done()
        .onTrue(
            Commands.sequence(
                SuperStructure.L3.get(),
                EndEffector.ALGAE_GRAB_AND_CORAL_SCORE.get(),
                Commands.parallel(
                    SuperStructure.PROCESSOR_PREP.get().beforeStarting(Commands.waitSeconds(0.5)),
                    reefEToProcTraj.cmd())));

    // When the trajectory is done, score in processor. Then go to source
    reefEToProcTraj
        .done()
        .onTrue(
            Commands.sequence(
                SuperStructure.PROCESSOR_PREP.get(),
                EndEffector.PROCESSOR_SCORE.get(),
                Commands.parallel(procToSourceTraj.cmd(), SuperStructure.STARTING_CONF.get())));

    // When at the source, pick up coral and go to reef B
    procToSourceTraj
        .done()
        .onTrue(Commands.sequence(SuperStructure.SOURCE_CORAL_INTAKE.get(), sourceToReefC.cmd()));

    // Prep elevator along the way
    sourceToReefC.atTime("PrepElevator").onTrue(SuperStructure.L3.get());

    // Once at reef B, score and pick up algae; then go to processor
    sourceToReefC
        .done()
        .onTrue(
            Commands.sequence(
                SuperStructure.L3.get(),
                EndEffector.ALGAE_GRAB_AND_CORAL_SCORE.get(),
                Commands.parallel(
                    SuperStructure.PROCESSOR_PREP.get().beforeStarting(Commands.waitSeconds(0.5)),
                    reefCToProcessor.cmd())));

    // When at the processor, score!
    reefCToProcessor.done().onTrue(EndEffector.PROCESSOR_SCORE.get());

    return routine;
  }
}
