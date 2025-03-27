// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.superstructure.EndEffector;
import frc.robot.commands.superstructure.SuperStructure;

/* Please use this class when testing out new commands and triggers
 * This will help keep the existing commands clean and we only move
 * things to driver or operator controls after testing
 */
public class DeveloperControls {
  private final CommandXboxController devCommandXboxController = new CommandXboxController(2);

  public void configureTriggers() {}

  public void configureButtonBindings() {

    devCommandXboxController.a().onTrue(SuperStructure.ALGAE_GRAB_GROUND.get());
    devCommandXboxController.b().onTrue(SuperStructure.ALGAE_GRAB_L2.get());
    devCommandXboxController.y().onTrue(SuperStructure.ALGAE_GRAB_L3.get());
    devCommandXboxController.rightBumper().onTrue(SuperStructure.PROCESSOR_SCORE.get());
    devCommandXboxController
        .x()
        .onTrue(Commands.sequence(SuperStructure.BARGE_PREP.get(), EndEffector.BARGE_SCORE.get()));
    devCommandXboxController.leftBumper().onTrue(SuperStructure.STARTING_CONF_WITH_ALGAE.get());
  }
}
