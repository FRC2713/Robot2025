package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;
import frc.robot.SetpointConstants;
import frc.robot.commands.ClimberCmds;
import frc.robot.commands.climber.MoveClimber;
import frc.robot.commands.superstructure.SuperStructure;

public class OperatorControls {
  private static final CommandXboxController operator = new CommandXboxController(1);

  // TODO: test with reef tracker (needs uncommited code from driver station)
  Trigger climbPrepTrigger = new Trigger(() -> RobotContainer.climbAssist.shouldClimbPrep());

  public void configureTriggers() {
    climbPrepTrigger.onTrue(SuperStructure.CLIMBING_CONF.get());
  }

  public void configureButtonBindings() {

    // Operator Controls
    operator.a().onTrue(SuperStructure.L1.get());
    operator.b().onTrue(SuperStructure.L2.get());
    operator.y().onTrue(SuperStructure.L3.get());
    operator.rightBumper().onTrue(SuperStructure.L4.get());

    operator.leftBumper().onTrue(SuperStructure.STARTING_CONF.get());

    // super structure control for climbing, also sets the driving mode to slow
    operator.start().onTrue(SuperStructure.CLIMBING_CONF.get());

    // climber control for climbing
    operator
        .leftTrigger(0.1)
        .whileTrue(new MoveClimber(operator::getLeftTriggerAxis))
        .onFalse(ClimberCmds.setVoltage(() -> 0));
    operator
        .rightTrigger(0.1)
        .whileTrue(
            Commands.sequence(
                Commands.either(
                    ClimberCmds.configureSoftLimits(
                        SetpointConstants.Climber.MIN_ANGLE_CLIMBING,
                        SetpointConstants.Climber.MAX_ANGLE_CLIMBING),
                    Commands.none(),
                    () -> RobotContainer.climber.getCurrentAngle() > 100),
                new MoveClimber(() -> -1 * operator.getRightTriggerAxis())))
        .onFalse(ClimberCmds.setVoltage(() -> 0));
  }
}
