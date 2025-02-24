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

  //   public AutoRoutine processAlgae() {
  //     String name = "Process Algae"
  //   }

  public AutoRoutine coralAndAlgaeAuto() {
    String name = "Coral and Algae Auto";
    AutoRoutine routine = m_factory.newRoutine(name);

    // Load the routine's trajectories
    AutoTrajectory startToReefDTraj = routine.trajectory("StartToReefD");
    AutoTrajectory reefDToProcTraj = routine.trajectory("ReefDToProcessor");
    AutoTrajectory procToSourceTraj = routine.trajectory("ProcessorToSource");
    AutoTrajectory sourceToReefB = routine.trajectory("SourceToReefB");
    AutoTrajectory reefBToProcessor = routine.trajectory("ReefBToProcessor");

    // When the routine begins, reset odometry and start the first trajectory
    routine
        .active()
        .onTrue(
            Commands.sequence(
                Commands.print(name + " started!"),
                startToReefDTraj.resetOdometry(),
                startToReefDTraj.cmd()));

    // Go from start to reef D, preping the elevator along the way
    startToReefDTraj.atTime("PrepElevator").onTrue(SuperStructure.L3_CORAL_PREP.getCommand());

    // When at the reef, score and pick up algae; then go to the processor
    startToReefDTraj
        .done()
        .onTrue(
            Commands.sequence(
                SuperStructure.L3_CORAL_SCORE.getCommand(),
                SuperStructure.L3_ALGAE_GRAB.getCommand(),
                Commands.parallel(
                    SuperStructure.PROCESSOR_PREP.getCommand(), reefDToProcTraj.cmd())));

    // When the trajectory is done, score in processor. Then go to source
    reefDToProcTraj
        .done()
        .onTrue(
            Commands.sequence(
                SuperStructure.PROCESSOR_SCORE.getCommand(),
                Commands.parallel(
                    procToSourceTraj.cmd(), SuperStructure.STARTING_CONF.getCommand())));

    // When at the source, pick up coral and go to reef B
    procToSourceTraj
        .done()
        .onTrue(
            Commands.sequence(
                SuperStructure.SOURCE_CORAL_INTAKE.getCommand(), sourceToReefB.cmd()));

    // Prep elevator along the way
    sourceToReefB.atTime("PrepElevator").onTrue(SuperStructure.L3_CORAL_PREP.getCommand());

    // Once at reef B, score and pick up algae; then go to processor
    sourceToReefB
        .done()
        .onTrue(
            Commands.sequence(
                SuperStructure.L3_CORAL_SCORE.getCommand(),
                SuperStructure.L3_ALGAE_GRAB.getCommand(),
                Commands.parallel(
                    SuperStructure.PROCESSOR_PREP.getCommand(), reefBToProcessor.cmd())));

    // When at the processor, score!
    reefBToProcessor.done().onTrue(SuperStructure.PROCESSOR_SCORE.getCommand());

    return routine;
  }

  public AutoRoutine scoreLotsOfCoral() {
    AutoRoutine routine = m_factory.newRoutine("Score Lots of Coral");

    // Load the routine's trajectories
    AutoTrajectory startToReefDTraj = routine.trajectory("Start2ToReefD");
    AutoTrajectory reefDToSource = routine.trajectory("ReefDToSource");
    AutoTrajectory sourceToReefB = routine.trajectory("SourceToReefB");
    AutoTrajectory reefBToSource = routine.trajectory("ReefBToSource");
    AutoTrajectory sourceToReefA = routine.trajectory("SourceToReefA");
    AutoTrajectory reefAToSource = routine.trajectory("ReefAToSource");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                new InstantCommand(() -> System.out.println("Score Lots of Coral started")),
                startToReefDTraj.resetOdometry(),
                startToReefDTraj.cmd()));

    // Go from start to reef D, preping the elevator along the way
    startToReefDTraj.atTime("PrepElevator").onTrue(SuperStructure.L3_CORAL_PREP.getCommand());
    // When at the reef, score and go to source
    startToReefDTraj
        .done()
        .onTrue(Commands.sequence(SuperStructure.L3_CORAL_SCORE.getCommand(), reefDToSource.cmd()));

    // When the trajectory is done, intake; then go to reef B
    reefDToSource
        .done()
        .onTrue(
            Commands.sequence(
                SuperStructure.SOURCE_CORAL_INTAKE.getCommand(), sourceToReefB.cmd()));

    // Prep elevator along the way
    sourceToReefB.atTime("PrepElevator").onTrue(SuperStructure.L3_CORAL_PREP.getCommand());
    // Once at reef B, score and go to source
    sourceToReefB
        .done()
        .onFalse(
            Commands.sequence(SuperStructure.L3_CORAL_SCORE.getCommand(), reefBToSource.cmd()));

    // When the trajectory is done, intake; then go to reef A
    reefBToSource
        .done()
        .onTrue(
            Commands.sequence(
                SuperStructure.SOURCE_CORAL_INTAKE.getCommand(), sourceToReefA.cmd()));

    // Prep elevator along the way
    sourceToReefA.atTime("PrepElevator").onTrue(SuperStructure.L3_CORAL_SCORE.getCommand());
    // Once at reef A, score and go to source
    sourceToReefA
        .done()
        .onFalse(
            Commands.sequence(SuperStructure.L3_CORAL_SCORE.getCommand(), reefAToSource.cmd()));

    reefAToSource.done().onTrue(SuperStructure.SOURCE_CORAL_INTAKE.getCommand());

    return routine;
  }
}
