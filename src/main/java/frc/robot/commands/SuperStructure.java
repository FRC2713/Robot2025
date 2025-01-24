package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.littletonrobotics.junction.Logger;

public final class SuperStructure {

  private static Command logStructure(String str) {
    // TODO: recordOutput misses some prints
    return Commands.sequence(
        new InstantCommand(() -> Logger.recordOutput("SuperStructureConsole", str)),
        Commands.print(str));
  }

  private static Command runStructure(String name, Command cmd) {
    return new SequentialCommandGroup(logStructure("Setting superstructure: " + name), cmd);
  }

  private static String treeifyReason(String reason) {
    return reason != "" ? " - " + reason + "/" : "";
  }

  public static Command STARTING_CONF() {
    return runStructure(
        "Starting conf",
        new SequentialCommandGroup(
            ElevatorCmds.setHeightCmd(0),
            PivotCmds.setAngle(0),
            RollerCmds.setAlgaeSpeedAndWait(0),
            ElevatorCmds.waitUntilAtTarget(),
            RollerCmds.waitUntilAlgaeAtTarget()));
  }

  public static Command L1_CORAL_PREP_ELEVATOR() {
    return L1_CORAL_PREP_ELEVATOR("");
  }

  public static Command L1_CORAL_PREP_ELEVATOR(String reason) {
    return runStructure(
        treeifyReason(reason) + "Prep Elevator",
        new SequentialCommandGroup(ElevatorCmds.setHeightWaitCmd(10)));
  }

  public static Command L1_CORAL_SCORE() {
    return L1_CORAL_SCORE("");
  }

  public static Command L1_CORAL_SCORE(String reason) {
    return runStructure(
        treeifyReason(reason) + "L1 Coral Score",
        new SequentialCommandGroup(
            L1_CORAL_PREP_ELEVATOR(treeifyReason(reason) + "L1 Coral Score"),
            RollerCmds.setTubeSpeed(1000),
            Commands.waitSeconds(1)));
  }

  public static Command L1_ALGAE_TAKE() {
    return L1_ALGAE_TAKE("");
  }

  public static Command L1_ALGAE_TAKE(String reason) {
    return runStructure(
        treeifyReason(reason) + "L1 Algae Take",
        new SequentialCommandGroup(
            L1_CORAL_PREP_ELEVATOR(treeifyReason(reason) + "L1 Algae Take"),
            RollerCmds.setAlgaeSpeed(1000),
            Commands.waitSeconds(1)));
  }

  public static Command L1_CORAL_SCORE_AND_ALGAE_TAKE() {
    return runStructure(
        "L1 Coral&Algae",
        new ParallelCommandGroup(
            L1_ALGAE_TAKE("L1 Coral&Algae"), L1_CORAL_SCORE("L1 Coral&Algae")));
  }

  public static Command PROCESSOR_PREP() {
    return PROCESSOR_PREP("");
  }

  public static Command PROCESSOR_PREP(String reason) {
    return runStructure(
        treeifyReason(reason) + "Processor prep",
        new ParallelCommandGroup(ElevatorCmds.setHeightCmd(0), PivotCmds.setAngleAndWait(10)));
  }

  public static Command PROCESSOR_SCORE() {
    return runStructure(
        "Processor score",
        new SequentialCommandGroup(
            PROCESSOR_PREP("Processor score"), RollerCmds.setAlgaeSpeed(-1000)));
  }

  public static Command SOURCE_PICK_UP() {
    return runStructure(
        "Source pick up",
        new SequentialCommandGroup(
            new ParallelCommandGroup(ElevatorCmds.setHeightCmd(0), PivotCmds.setAngle(0)),
            // new WaitUntilCommand(CoralThing::hasCoral);
            new WaitCommand(2)));
  }
}
