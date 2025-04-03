package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.superstructure.EndEffector;
import frc.robot.commands.superstructure.SuperStructure;
import java.util.function.Supplier;
import lombok.Getter;

public enum ScoreLevel {
  ONE(Commands::none, SuperStructure.L1, EndEffector.CORAL_SCORE),
  TWO(SuperStructure.L2, SuperStructure.L2, EndEffector.CORAL_SCORE),
  ALGAE_TWO(
      SuperStructure.ALGAE_GRAB_L2, SuperStructure.ALGAE_GRAB_L2, SuperStructure.ALGAE_COLLECT_L2),
  THREE(SuperStructure.L3, SuperStructure.L3, EndEffector.CORAL_SCORE),
  ALGAE_THREE(
      SuperStructure.ALGAE_GRAB_L3, SuperStructure.ALGAE_GRAB_L3, SuperStructure.ALGAE_COLLECT_L3),
  FOUR(
      SuperStructure.L4_PREP,
      SuperStructure.L4,
      () ->
          Commands.either(
              EndEffector.CORAL_SCORE.get().beforeStarting(Commands.waitSeconds(0.3)),
              EndEffector.CORAL_SCORE.get().beforeStarting(Commands.waitSeconds(0.1)),
              DriverStation::isAutonomous));

  @Getter private Supplier<Command> prepCommand;
  @Getter private Supplier<Command> ssCommand;
  @Getter private Supplier<Command> scoreCommand;

  private ScoreLevel(
      Supplier<Command> prepCommand, Supplier<Command> ssCommand, Supplier<Command> scoreCommand) {
    this.prepCommand = prepCommand;
    this.ssCommand = ssCommand;
    this.scoreCommand = scoreCommand;
  }

  // public static Map<ScoreLevel, Command> createPrepCommandsMap() {
  //   var cmdsMap = new HashMap<ScoreLevel, Command>();
  //   for (ScoreLevel level : ScoreLevel.values()) {
  //     cmdsMap.put(level, level.getPrepCommand());
  //   }
  //   return cmdsMap;
  // }

  // public static Map<ScoreLevel, Command> createScoreCommandsMap() {
  //   var cmdsMap = new HashMap<ScoreLevel, Command>();
  //   for (ScoreLevel level : ScoreLevel.values()) {
  //     cmdsMap.put(level, level.getScoreCommand());
  //   }
  //   return cmdsMap;
  // }
}
