package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;
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
    operator.a().onTrue(SuperStructure.L1.get());
    operator.b().onTrue(SuperStructure.L2.get());
    operator.y().onTrue(SuperStructure.L3.get());
    operator.rightBumper().onTrue(SuperStructure.L4.get());

    operator.leftBumper().onTrue(SuperStructure.STARTING_CONF.get());
  }
}
