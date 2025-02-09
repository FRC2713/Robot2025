package frc.robot.commands.autos;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.SuperStructure;

public class AutoRoutines {
  private final AutoFactory m_factory;

  public AutoRoutines(AutoFactory factory) {
    m_factory = factory;
  }

  public AutoRoutine coralAndAlgaeAuto() {
    String name = "Coral and Algae Auto";
    AutoRoutine routine = m_factory.newRoutine(name);

    // Load the routine's trajectories
    AutoTrajectory startToReefB2Traj = routine.trajectory("StartToReefB2");
    AutoTrajectory reefB2ToProcTraj = routine.trajectory("ReefB2ToProcessor");
    AutoTrajectory procToSourceTraj = routine.trajectory("ProcessorToSource");
    AutoTrajectory sourceToReefA2 = routine.trajectory("SourceToReefA2");
    AutoTrajectory reefA2ToProcessor = routine.trajectory("ReefA2ToProcessor");

    // When the routine begins, reset odometry and start the first trajectory
    routine
        .active()
        .onTrue(
            Commands.sequence(
                Commands.print(name + " started!"),
                startToReefB2Traj.resetOdometry(),
                startToReefB2Traj.cmd()));

    // Go from start to reef B2, preping the elevator along the way
    startToReefB2Traj.atTime("PrepElevator").onTrue(SuperStructure.L3_CORAL_PREP.getCommand());

    // When at the reef, score and pick up algae; then go to the processor
    startToReefB2Traj
        .done()
        .onTrue(
            Commands.sequence(
                SuperStructure.L3_CORAL_SCORE.getCommand(),
                SuperStructure.L3_ALGAE_GRAB.getCommand(),
                SuperStructure.STARTING_CONF.getCommand(),
                reefB2ToProcTraj.cmd()));

    // When the trajectory is done, score in processor. Then go to source
    reefB2ToProcTraj
        .done()
        .onTrue(
            Commands.sequence(SuperStructure.PROCESSOR_SCORE.getCommand(), procToSourceTraj.cmd()));

    // When at the source, pick up coral and go to reef A2
    procToSourceTraj
        .done()
        .onTrue(
            Commands.sequence(
                SuperStructure.SOURCE_CORAL_INTAKE.getCommand(), sourceToReefA2.cmd()));

    // Prep elevator along the way
    sourceToReefA2.atTime("PrepElevator").onTrue(SuperStructure.L3_CORAL_PREP.getCommand());

    // Once at reef A2, score and pick up algae; then go to processor
    sourceToReefA2
        .done()
        .onFalse(
            Commands.sequence(
                SuperStructure.L3_CORAL_SCORE.getCommand(),
                SuperStructure.L3_ALGAE_GRAB.getCommand(),
                reefA2ToProcessor.cmd()));

    // When at the processor, score!
    reefA2ToProcessor.done().onTrue(SuperStructure.PROCESSOR_SCORE.getCommand());

    return routine;
  }

  public AutoRoutine scoreLotsOfCoral() {
    AutoRoutine routine = m_factory.newRoutine("Score Lots of Coral");

    // Load the routine's trajectories
    AutoTrajectory startToReefB2Traj = routine.trajectory("Start2ToReefB2");
    AutoTrajectory reefB2ToSource = routine.trajectory("ReefB2ToSource");
    AutoTrajectory sourceToReefA2 = routine.trajectory("SourceToReefA2");
    AutoTrajectory reefA2ToSource = routine.trajectory("ReefA2ToSource");
    AutoTrajectory sourceToReefA1 = routine.trajectory("SourceToReefA1");
    AutoTrajectory reefA1ToSource = routine.trajectory("ReefA1ToSource");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                new InstantCommand(() -> System.out.println("Score Lots of Coral started")),
                startToReefB2Traj.resetOdometry(),
                startToReefB2Traj.cmd()));

    // Go from start to reef B2, preping the elevator along the way
    startToReefB2Traj.atTime("PrepElevator").onTrue(SuperStructure.L3_CORAL_PREP.getCommand());
    // When at the reef, score and go to source
    startToReefB2Traj
        .done()
        .onTrue(
            Commands.sequence(SuperStructure.L3_CORAL_SCORE.getCommand(), reefB2ToSource.cmd()));

    // When the trajectory is done, intake; then go to reef A2
    reefB2ToSource
        .done()
        .onTrue(
            Commands.sequence(
                SuperStructure.SOURCE_CORAL_INTAKE.getCommand(), sourceToReefA2.cmd()));

    // Prep elevator along the way
    sourceToReefA2.atTime("PrepElevator").onTrue(SuperStructure.L3_CORAL_PREP.getCommand());
    // Once at reef A2, score and go to source
    sourceToReefA2
        .done()
        .onFalse(
            Commands.sequence(SuperStructure.L3_CORAL_SCORE.getCommand(), reefA2ToSource.cmd()));

    // When the trajectory is done, intake; then go to reef A1
    reefA2ToSource
        .done()
        .onTrue(
            Commands.sequence(
                SuperStructure.SOURCE_CORAL_INTAKE.getCommand(), sourceToReefA1.cmd()));

    // Prep elevator along the way
    sourceToReefA1.atTime("PrepElevator").onTrue(SuperStructure.L3_CORAL_SCORE.getCommand());
    // Once at reef A1, score and go to source
    sourceToReefA1
        .done()
        .onFalse(
            Commands.sequence(SuperStructure.L3_CORAL_SCORE.getCommand(), reefA1ToSource.cmd()));

    reefA1ToSource.done().onTrue(SuperStructure.SOURCE_CORAL_INTAKE.getCommand());

    return routine;
  }
}
