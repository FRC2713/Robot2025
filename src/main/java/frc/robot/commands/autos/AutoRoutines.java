package frc.robot.commands.autos;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class AutoRoutines {
  private final AutoFactory m_factory;

  public AutoRoutines(AutoFactory factory) {
    m_factory = factory;
  }

  public AutoRoutine exampleAuto() {
    AutoRoutine routine = m_factory.newRoutine("Example Auto");

    // Load the routine's trajectories
    AutoTrajectory startToReefTraj = routine.trajectory("StartToReef");
    AutoTrajectory reefToProcTraj = routine.trajectory("ReefB2ToProcessorToSource");
    AutoTrajectory sourceToReefA2 = routine.trajectory("SourceToReefA2");
    AutoTrajectory reefA2ToProcessor = routine.trajectory("ReefA2ToProcessor");

    // When the routine begins, reset odometry and start the first trajectory
    routine
        .active()
        .onTrue(
            Commands.sequence(
                new InstantCommand(() -> System.out.println("Example Auto started")),
                startToReefTraj.resetOdometry(),
                startToReefTraj.cmd()));

    // Starting at the event marker named "intake", run the intake
    // startToReefTraj.atTime("PrepElevator").onTrue(SuperStructure.L1_CORAL_PREP_ELEVATOR());

    // // When the trajectory is done, start the next trajectory
    startToReefTraj
        .done()
        .onTrue(
            Commands.sequence(
                // SuperStructure.L1_CORAL_SCORE_AND_ALGAE_TAKE(),
                reefToProcTraj.cmd()));

    // // While the trajectory is active, prepare the scoring subsystem
    // reefToProcTraj.active().whileTrue(SuperStructure.PROCESSOR_PREP());
    // reefToProcTraj.atTime("PrepProcessor").onTrue(SuperStructure.PROCESSOR_PREP());

    // reefToProcTraj.atTime("ScoreProcessor").onTrue(SuperStructure.PROCESSOR_SCORE());

    // // When the trajectory is done, score
    reefToProcTraj
        .done()
        .onTrue(
            Commands.sequence(
                // SuperStructure.SOURCE_PICK_UP(),
                sourceToReefA2.cmd()));

    sourceToReefA2
        .done()
        .onFalse(
            Commands.sequence(
                // SuperStructure.L1_CORAL_SCORE_AND_ALGAE_TAKE(),
                reefA2ToProcessor.cmd()));

    // reefA2ToProcessor.atTime("PrepProcessor").onTrue(SuperStructure.PROCESSOR_PREP());

    // reefA2ToProcessor.done().onTrue(SuperStructure.PROCESSOR_SCORE());

    return routine;
  }

  public AutoRoutine scoreLotsOfCoral() {
    AutoRoutine routine = m_factory.newRoutine("Score Lots of Coral");

    // Load the routine's trajectories
    AutoTrajectory startToReefTraj = routine.trajectory("Start2ToReefB2");
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
                startToReefTraj.resetOdometry(),
                startToReefTraj.cmd()));

    // startToReefTraj.atTime("PrepElevator").onTrue(SuperStructure.L1_CORAL_PREP_ELEVATOR());
    startToReefTraj
        .done()
        .onTrue(
            Commands.sequence(
                // SuperStructure.L1_CORAL_SCORE(),
                reefB2ToSource.cmd()));

    reefB2ToSource
        .done()
        .onTrue(
            Commands.sequence(
                // SuperStructure.SOURCE_PICK_UP(),
                sourceToReefA2.cmd()));

    // sourceToReefA2.atTime("PrepElevator").onTrue(SuperStructure.L1_CORAL_PREP_ELEVATOR());
    sourceToReefA2
        .done()
        .onFalse(
            Commands.sequence(
                // SuperStructure.L1_CORAL_SCORE(),
                reefA2ToSource.cmd()));

    reefA2ToSource
        .done()
        .onTrue(
            Commands.sequence(
                // SuperStructure.SOURCE_PICK_UP(),
                sourceToReefA1.cmd()));

    // sourceToReefA1.atTime("PrepElevator").onTrue(SuperStructure.L1_CORAL_PREP_ELEVATOR());
    sourceToReefA1
        .done()
        .onFalse(
            Commands.sequence(
                // SuperStructure.L1_CORAL_SCORE(),
                reefA1ToSource.cmd()));

    return routine;
  }
}
