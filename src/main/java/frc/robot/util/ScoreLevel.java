package frc.robot.util;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.SuperStructure;
import lombok.Getter;

public enum ScoreLevel {
  ONE(SuperStructure.L1_CORAL_PREP_ELEVATOR(), SuperStructure.L1_CORAL_SCORE()),
  // TODO: Different commands for different levels
  TWO(SuperStructure.L1_CORAL_PREP_ELEVATOR(), SuperStructure.L1_CORAL_SCORE()),
  THREE(SuperStructure.L1_CORAL_PREP_ELEVATOR(), SuperStructure.L1_CORAL_SCORE()),
  FOUR(SuperStructure.L1_CORAL_PREP_ELEVATOR(), SuperStructure.L1_CORAL_SCORE());

  @Getter private Command prepCommand;
  @Getter private Command scoreCommand;

  private ScoreLevel(Command prepCommand, Command scoreCommand) {
    this.prepCommand = prepCommand;
    this.scoreCommand = scoreCommand;
  }
}
