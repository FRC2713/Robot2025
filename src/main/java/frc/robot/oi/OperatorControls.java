package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;
import frc.robot.SSConstants;
import frc.robot.commands.ClimberCmds;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ScoreAssist;
import frc.robot.commands.SuperStructure;
import frc.robot.commands.climber.MoveClimber;
import frc.robot.subsystems.climber.Climber;

public class OperatorControls {
  private static final CommandXboxController operator = new CommandXboxController(1);
  private Trigger climbPrepTrigger = new Trigger(ScoreAssist.getInstance()::shouldClimbPrep);
  private final DriverControls driver;
  private final Climber climber;

  public OperatorControls(DriverControls driver, Climber climber) {
    this.driver = driver;
    this.climber = climber;
  }

  public void configureButtonBindings() {
    // Operator Controls
    operator.a().onTrue(SuperStructure.L1.getCommand());
    operator.b().onTrue(SuperStructure.L2.getCommand());
    operator.y().onTrue(SuperStructure.L3.getCommand());
    operator.rightBumper().onTrue(SuperStructure.L4.getCommand());

    climbPrepTrigger.onTrue(
        Commands.parallel(
            DriveCommands.changeDefaultDriveCommand(
                RobotContainer.driveSubsystem,
                DriveCommands.joystickDriveSlow(
                    RobotContainer.driveSubsystem,
                    () -> -driver.getLeftY(),
                    () -> -driver.getLeftX(),
                    () -> -driver.getRightX()),
                "Slow Control"),
            SuperStructure.CLIMBING_CONF.getCommand()));

    operator
        .start()
        .onTrue(
            Commands.parallel(
                DriveCommands.changeDefaultDriveCommand(
                    RobotContainer.driveSubsystem,
                    DriveCommands.joystickDriveSlow(
                        RobotContainer.driveSubsystem,
                        () -> -driver.getLeftY(),
                        () -> -driver.getLeftX(),
                        () -> -driver.getRightX()),
                    "Slow Control"),
                SuperStructure.CLIMBING_CONF.getCommand()));
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
                        SSConstants.Climber.MIN_ANGLE_CLIMBING,
                        SSConstants.Climber.MAX_ANGLE_CLIMBING),
                    Commands.none(),
                    () -> climber.getCurrentAngle() > 100),
                new MoveClimber(() -> -1 * operator.getRightTriggerAxis())))
        .onFalse(ClimberCmds.setVoltage(() -> 0));

    operator.leftBumper().onTrue(SuperStructure.STARTING_CONF.getCommand());
  }
}
