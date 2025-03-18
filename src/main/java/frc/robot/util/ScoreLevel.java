package frc.robot.util;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.scoreassist.SuperStructure;
import java.util.function.Supplier;
import lombok.Getter;

public enum ScoreLevel {
  ONE(Commands::none, SuperStructure.L1::getCommand),
  TWO(SuperStructure.L2::getCommand, SuperStructure.L2::getCommand),
  THREE(SuperStructure.L3::getCommand, SuperStructure.L3::getCommand),
  FOUR(SuperStructure.L4_PREP::getCommand, SuperStructure.L4::getCommand);

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
