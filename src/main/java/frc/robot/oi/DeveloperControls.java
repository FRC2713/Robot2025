// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/* Please use this class when testing out new commands and triggers
 * This will help keep the existing commands clean and we only move 
 * things to driver or operator controls after testing
 */
public class DeveloperControls {
      private final CommandXboxController devCommandXboxController = new CommandXboxController(3);

      public void configureTriggers() {

      }

      public void configureButtonBindings() {

      }
}
