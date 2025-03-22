package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.SetpointConstants;
import frc.robot.commands.ClimberCmds;
import frc.robot.commands.ScoreAssist;
import frc.robot.commands.climber.MoveClimber;
import frc.robot.commands.superstructure.SuperStructure;
import frc.robot.subsystems.climber.Climber;

public class OperatorControls {
  private static final CommandXboxController operator = new CommandXboxController(1);
  private Trigger climbPrepTrigger = new Trigger(ScoreAssist.getInstance()::shouldClimbPrep);
  private final Climber climber;

  public OperatorControls(Climber climber) {
    this.climber = climber;
  }

  public void configureButtonBindings() {
    // Operator Controls
    operator.a().onTrue(SuperStructure.L1.get());
    operator.b().onTrue(SuperStructure.L2.get());
    operator.y().onTrue(SuperStructure.L3.get());
    operator.rightBumper().onTrue(SuperStructure.L4.get());

    operator.leftBumper().onTrue(SuperStructure.STARTING_CONF.get());

    // super structure control for climbing
    operator.start().onTrue(SuperStructure.CLIMBING_CONF.get());
    climbPrepTrigger.onTrue(SuperStructure.CLIMBING_CONF.get());

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
                    () -> climber.getCurrentAngle() > 100),
                new MoveClimber(() -> -1 * operator.getRightTriggerAxis())))
        .onFalse(ClimberCmds.setVoltage(() -> 0));
  }
}
