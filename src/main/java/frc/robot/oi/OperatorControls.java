package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;
import frc.robot.commands.ArmCmds;
import frc.robot.commands.superstructure.SuperStructure;

public class OperatorControls {
  private static final CommandXboxController operator = new CommandXboxController(1);

  // TODO: test with reef tracker (needs uncommited code from driver station)
  Trigger climbPrepTrigger = new Trigger(() -> RobotContainer.climbAssist.shouldClimbPrep());

  public void configureTriggers() {
    /*climbPrepTrigger.onTrue(
    Commands.sequence(
        Commands.runOnce(() -> SmartDashboard.putBoolean("Disable ReefAlign", true)),
        SuperStructure.CLIMBING_CONF.get()));*/
  }

  public void configureButtonBindings() {

    // Operator Controls
    operator.a().onTrue(SuperStructure.ALGAE_GRAB_L2.get()).onFalse(ArmCmds.handSetVoltage(-1));
    operator.b().onTrue(SuperStructure.ALGAE_GRAB_L3.get()).onFalse(ArmCmds.handSetVoltage(-1));
    operator.y().onTrue(SuperStructure.ALGAE_SCORE.get()).onFalse(ArmCmds.handSetVoltage(-1));
    operator.rightBumper().onTrue(SuperStructure.L4.get());

    operator.start().onTrue(SuperStructure.UNFOLD.get());
    operator.back().onTrue(SuperStructure.FOLD.get());

    operator.leftBumper().onTrue(SuperStructure.STARTING_CONF.get());
  }
}
