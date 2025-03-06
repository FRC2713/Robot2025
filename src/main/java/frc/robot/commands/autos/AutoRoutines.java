package frc.robot.commands.autos;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.commands.ScoreAssist;
import frc.robot.commands.SuperStructure;
import frc.robot.util.AllianceFlipUtil;

public class AutoRoutines {
  private final AutoFactory m_factory;

  public AutoRoutines(AutoFactory factory) {
    m_factory = factory;
  }

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
    startToReefDTraj.atTime("PrepElevator").onTrue(SuperStructure.L3_PREP.getCommand());

    // When at the reef, score and pick up algae; then go to the processor
    startToReefDTraj
        .done()
        .onTrue(
            Commands.sequence(
                SuperStructure.L3_PREP.getCommand(),
                SuperStructure.ALGAE_GRAB_AND_CORAL_SCORE.getCommand(),
                Commands.parallel(
                    SuperStructure.PROCESSOR_PREP.delayCommand(0.5), reefDToProcTraj.cmd())));

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
    sourceToReefB.atTime("PrepElevator").onTrue(SuperStructure.L3_PREP.getCommand());

    // Once at reef B, score and pick up algae; then go to processor
    sourceToReefB
        .done()
        .onTrue(
            Commands.sequence(
                SuperStructure.L3_PREP.getCommand(),
                SuperStructure.ALGAE_GRAB_AND_CORAL_SCORE.getCommand(),
                Commands.parallel(
                    SuperStructure.PROCESSOR_PREP.delayCommand(0.5), reefBToProcessor.cmd())));

    // When at the processor, score!
    reefBToProcessor.done().onTrue(SuperStructure.PROCESSOR_SCORE.getCommand());

    return routine;
  }

  public AutoRoutine scoreLotsOfCoralGeneratedLeg1() {
    String name = "Score Lots of Coral";
    AutoRoutine routine = m_factory.newRoutine(name);

    // 1) Load the routine's trajectories

    // AutoTrajectory startToReefDTraj = routine.trajectory("Start2ToReefD");
    AutoTrajectory reefDToSource = routine.trajectory("ReefDToSource");
    AutoTrajectory sourceToReefB = routine.trajectory("SourceToReefB");
    AutoTrajectory reefBToSource = routine.trajectory("ReefBToSource");
    AutoTrajectory sourceToReefA = routine.trajectory("SourceToReefA");
    AutoTrajectory reefAToSource = routine.trajectory("ReefAToSource");

    // 1) Generate the a PathPlannerLib trajectory for the first leg. Go to reefD, then score on L4

    // when generating leg1, you don't need to reset odemetry at the start
    Command leg1 =
        Commands.sequence(
            ScoreAssist.buildOTFPath(
                AllianceFlipUtil.apply(reefDToSource.getInitialPose().get()),
                Constants.scoreAssistConstraints,
                0,
                false),
            SuperStructure.L4_PREP.getCommand(),
            SuperStructure.CORAL_SCORE.getCommand(),
            Commands.waitSeconds(0.2));

    // when the routine starts, first run the path-finding command
    routine
        .active()
        .onTrue(
            Commands.sequence(
                Commands.print(name + " started generated leg!"),
                leg1,
                Commands.waitSeconds(0.2),
                Commands.print(name + " started routine!"),
                // reefDToSource.resetOdometry(),
                SuperStructure.L4_PREP.getCommand(),
                Commands.waitSeconds(0.2),
                reefDToSource.cmd()));

    // 2) Run the rest of the routine


    // When the trajectory is done, intake; then go to reef B
    reefDToSource
        .done()
        .onTrue(
            Commands.parallel(
                SuperStructure.SOURCE_CORAL_INTAKE.getCommand(),
                Commands.sequence(Commands.waitSeconds(0.1), sourceToReefB.cmd())));

    // Prep elevator along the way
    // sourceToReefB.atTime("PrepElevator").onTrue(SuperStructure.L3_PREP.getCommand());
    // Once at reef B, score and go to source
    sourceToReefB
        .done()
        .onTrue(
            Commands.sequence(
                SuperStructure.L4_PREP.getCommand(),
                SuperStructure.CORAL_SCORE.getCommand(),
                reefBToSource.cmd()));

    // When the trajectory is done, intake; then go to reef A
    reefBToSource
        .done()
        .onTrue(
            Commands.sequence(
                SuperStructure.SOURCE_CORAL_INTAKE.getCommand(), sourceToReefA.cmd()));

    // Prep elevator along the way
    sourceToReefA.atTime("PrepElevator").onTrue(SuperStructure.L4_PREP.getCommand());
    // Once at reef A, score and go to source
    sourceToReefA
        .done()
        .onTrue(
            Commands.sequence(
                SuperStructure.L4_PREP.getCommand(),
                SuperStructure.CORAL_SCORE.getCommand(),
                reefAToSource.cmd()));

    reefAToSource.done().onTrue(SuperStructure.SOURCE_CORAL_INTAKE.getCommand());

    return routine;
  }

  public AutoRoutine driveStraightGeneratedLeg1() {
    String name = "DriveStraight";
    AutoRoutine routine = m_factory.newRoutine("DriveStraight");

    var startToReefTraj = routine.trajectory("Example");
    Command leg1 =
    Commands.sequence(
        ScoreAssist.buildOTFPath(
            AllianceFlipUtil.apply(startToReefTraj.getFinalPose().get()),
            Constants.scoreAssistConstraints,
            0,
            false),
        SuperStructure.L4_PREP.getCommand(),
        SuperStructure.CORAL_SCORE.getCommand(),
        Commands.waitSeconds(0.2));

    // When the routine begins, reset odometry and start the first trajectory
    routine
        .active()
        .onTrue(
            Commands.sequence(
                new InstantCommand(() -> System.out.println(name + " started")),
                leg1));

    return routine;
  }
}
