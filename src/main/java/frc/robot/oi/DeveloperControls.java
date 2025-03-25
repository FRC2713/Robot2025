// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;
import frc.robot.commands.superstructure.EndEffector;
import frc.robot.commands.superstructure.SuperStructure;

/* Please use this class when testing out new commands and triggers
 * This will help keep the existing commands clean and we only move 
 * things to driver or operator controls after testing
 */
public class DeveloperControls {
      private final CommandXboxController devCommandXboxController = new CommandXboxController(3);

      private Trigger hasAlgaeTrigger = new Trigger(() -> RobotContainer.endEffector.hasAlgae());
      public void configureTriggers() {
            hasAlgaeTrigger.onTrue(EndEffector.ALGAE_HOLD.get());
      }

      public void configureButtonBindings() {
            devCommandXboxController.b().onTrue(SuperStructure.ALGAE_GRAB_GROUND.get());
            devCommandXboxController.x().onTrue(SuperStructure.STARTING_CONF_WITH_ALGAE.get());
      }
}
