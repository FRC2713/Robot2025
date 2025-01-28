package frc.robot.util;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.SuperStructure;
import java.util.function.Supplier;
import lombok.Getter;

public enum ScoreLevel {
  ONE(SuperStructure::L1_CORAL_PREP_ELEVATOR, SuperStructure::L1_CORAL_SCORE),
  TWO(SuperStructure::L2_CORAL_PREP_ELEVATOR, SuperStructure::L2_CORAL_SCORE),
  // TODO: Different commands for different levels
  THREE(SuperStructure::L1_CORAL_PREP_ELEVATOR, SuperStructure::L1_CORAL_SCORE),
  FOUR(SuperStructure::L1_CORAL_PREP_ELEVATOR, SuperStructure::L1_CORAL_SCORE);

  @Getter private Supplier<Command> prepCommand;
  @Getter private Supplier<Command> scoreCommand;

  private ScoreLevel(Supplier<Command> prepCommand, Supplier<Command> scoreCommand) {
    this.prepCommand = prepCommand;
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
