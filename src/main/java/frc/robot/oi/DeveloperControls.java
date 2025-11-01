// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;
import frc.robot.SetpointConstants;
import frc.robot.commands.ArmCmds;
import frc.robot.commands.IntakeCmds;
import frc.robot.commands.superstructure.EndEffector;
import frc.robot.commands.superstructure.SuperStructure;

/* Please use this class when testing out new commands and triggers
 * This will help keep the existing commands clean and we only move
 * things to driver or operator controls after testing
 */
public class DeveloperControls {
  private final CommandXboxController devCommandXboxController = new CommandXboxController(2);

  private Trigger hasAlgaeTrigger = new Trigger(() -> RobotContainer.endEffector.hasAlgae());

  public void configureTriggers() {
    hasAlgaeTrigger.onTrue(EndEffector.ALGAE_HOLD.get());
  }

  public void configureButtonBindings() {

    // devCommandXboxController.a().onTrue(ArmCmds.handSetVoltage(15));
    // devCommandXboxController.b().onTrue(ArmCmds.armSetAngle(-20));
    // devCommandXboxController.y().onTrue(ArmCmds.armSetAngle(90));
    devCommandXboxController.a().onTrue(IntakeCmds.setAngle(SetpointConstants.Intake.INTAKE_GRAB_ANGLE));
    // devCommandXboxController.b().onTrue(IntakeCmds.setAngle(100));
    devCommandXboxController.y().onTrue(IntakeCmds.setAngle(80));
    devCommandXboxController
        .leftBumper()
        .whileTrue(SuperStructure.CORAL_GRAB_GROUND.get())
        .onFalse(SuperStructure.STARTING_CONF.get());
    devCommandXboxController.rightBumper().onTrue(ArmCmds.armSetAngle(-90));
    devCommandXboxController.leftTrigger().onTrue(IntakeCmds.setVolts(5));
    devCommandXboxController.rightTrigger().onTrue(IntakeCmds.setVolts(0));

    devCommandXboxController.povUp().onTrue(ArmCmds.handSetVoltage(5));
    devCommandXboxController.povDown().onTrue(SuperStructure.PRE_DISABLE_CHECK.get());
    // devCommandXboxController
    //     .rightBumper()
    //     .onTrue(
    //         Commands.sequence(
    //             SuperStructure.PROCESSOR_PREP.get(), EndEffector.PROCESSOR_SCORE.get()));
    // devCommandXboxController
    //     .x()
    //     .onTrue(
    //         Commands.sequence(
    //             SuperStructure.BARGE_PREP_FORWARDS.get(), EndEffector.BARGE_SCORE.get()));
    // devCommandXboxController.leftBumper().onTrue(SuperStructure.STARTING_CONF_WITH_ALGAE.get());
  }
}
