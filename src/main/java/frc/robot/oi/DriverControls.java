package frc.robot.oi;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;
import frc.robot.SetpointConstants;
import frc.robot.commands.ClimberCmds;
import frc.robot.commands.DriveCmds;
import frc.robot.commands.climber.Climb;
import frc.robot.commands.scoreassist.ScoreAssistCmds;
import frc.robot.commands.superstructure.EndEffector;
import frc.robot.commands.superstructure.SuperStructure;
import frc.robot.scoreassist.ReefAlign;
import frc.robot.scoreassist.SourceAlign;

public class DriverControls {
  private final CommandXboxController driver = new CommandXboxController(0);

  private Trigger reefAlignTrigger = new Trigger(ReefAlign.getInstance()::shouldDoReefAlign);
  private Trigger sourceAlignTrigger = new Trigger(SourceAlign.getInstance()::shouldDoSourceAlign);

  public void configureTriggers() {
    reefAlignTrigger.onTrue(this.setToReefAlignCmd()).onFalse(this.setToNormalDriveCmd());
    sourceAlignTrigger.onTrue(this.setToSourceAlignCmd()).onFalse(this.setToNormalDriveCmd());
  }

  public void configureButtonBindings() {

    // Reset gyro to 0 deg when start button is pressed
    driver
        .start()
        .onTrue(
            Commands.parallel(
                this.setToNormalDriveCmd(),
                Commands.runOnce(
                        () ->
                            RobotContainer.driveSubsystem.setPose(
                                new Pose2d(
                                    RobotContainer.driveSubsystem.getPose().getTranslation(),
                                    Rotation2d.fromDegrees(0))),
                        RobotContainer.driveSubsystem)
                    .ignoringDisable(true)));

    // Reset gyro to 180 deg when start button is pressed
    driver
        .back()
        .onTrue(
            Commands.parallel(
                    this.setToNormalDriveCmd(),
                    Commands.runOnce(
                        () ->
                            RobotContainer.driveSubsystem.setPose(
                                new Pose2d(
                                    RobotContainer.driveSubsystem.getPose().getTranslation(),
                                    Rotation2d.fromDegrees(180))),
                        RobotContainer.driveSubsystem))
                .ignoringDisable(true));

    // Intake Coral
    driver
        .leftBumper()
        .whileTrue(SuperStructure.SOURCE_CORAL_INTAKE.get())
        .onFalse(EndEffector.STOP_ROLLERS.get());

    // Score Coral w Score Assist
    driver
        .rightBumper()
        .onTrue(ScoreAssistCmds.exectuteCoralScore())
        .onFalse(ScoreAssistCmds.stop());

    // Enable/disable sourcealign
    driver
        .a()
        .onTrue(
            Commands.parallel(
                Commands.runOnce(() -> RobotContainer.disableSourceAlign = false),
                SuperStructure.SOURCE_CORAL_INTAKE.get()))
        .onFalse(
            Commands.parallel(
                Commands.runOnce(() -> RobotContainer.disableSourceAlign = true),
                EndEffector.STOP_ROLLERS.get()));

    // Enable/disable reefalign
    // driver
    //     .b()
    //     .onTrue(Commands.runOnce(() -> RobotContainer.disableReefAlign = false))
    //     .onFalse(Commands.runOnce(() -> RobotContainer.disableReefAlign = true));
    driver.b().onTrue(SuperStructure.ALGAE_GRAB_GROUND.get());
    driver.x().onTrue(SuperStructure.STARTING_CONF_WITH_ALGAE.get());
    // Climber
    driver
        .leftTrigger(0.1)
        .whileTrue(new Climb(() -> -1 * driver.getLeftTriggerAxis()))
        .onFalse(ClimberCmds.setVoltage(() -> 0));

    // Grab Algae
    driver
        .rightTrigger(0.2)
        .whileTrue(Commands.sequence(EndEffector.ALGAE_GRAB.get()))
        .onFalse(SuperStructure.STARTING_CONF_WITH_ALGAE.get());

    // POV Precision Driving
    driver
        .povLeft()
        .onTrue(
            DriveCmds.changeDefaultDriveCommand(
                RobotContainer.driveSubsystem,
                DriveCmds.inch(RobotContainer.driveSubsystem, SetpointConstants.Drive.INCH_SPEED),
                "Inch Left"))
        .onFalse(this.setToNormalDriveCmd());
    driver
        .povRight()
        .onTrue(
            DriveCmds.changeDefaultDriveCommand(
                RobotContainer.driveSubsystem,
                DriveCmds.inch(
                    RobotContainer.driveSubsystem,
                    () -> -1 * SetpointConstants.Drive.INCH_SPEED.getAsDouble()),
                "Inch Right"))
        .onFalse(this.setToNormalDriveCmd());

    // POV/x heading controller
    // driver
    //     .povLeft()
    //     .onTrue(
    //         DriveCmds.changeDefaultDriveCommand(
    //             RobotContainer.driveSubsystem,
    //             DriveCmds.joystickDriveAtAngle(
    //                 RobotContainer.driveSubsystem,
    //                 () -> -driver.getLeftY(),
    //                 () -> -driver.getLeftX(),
    //                 () -> Rotation2d.fromDegrees(0)),
    //             "Heading Controller"))
    //     .onFalse(this.setToNormalDriveCmd());
    // driver
    //     .povRight()
    //     .onTrue(
    //         DriveCmds.changeDefaultDriveCommand(
    //             RobotContainer.driveSubsystem,
    //             DriveCmds.joystickDriveAtAngle(
    //                 RobotContainer.driveSubsystem,
    //                 () -> -driver.getLeftY(),
    //                 () -> -driver.getLeftX(),
    //                 () -> Rotation2d.fromDegrees(180)),
    //             "Heading Controller"))
    //     .onFalse(this.setToNormalDriveCmd());
    // driver
    //     .x()
    //     .onTrue(
    //         DriveCmds.changeDefaultDriveCommand(
    //             RobotContainer.driveSubsystem,
    //             DriveCmds.joystickDriveAtAngle(
    //                 RobotContainer.driveSubsystem,
    //                 () -> -driver.getLeftY(),
    //                 () -> -driver.getLeftX(),
    //                 () ->
    // Rotation2d.fromDegrees(RobotContainer.driveSubsystem.getAngleToReef())),
    //             "Heading Controller"))
    //     .onFalse(this.setToNormalDriveCmd());
  }

  public double getLeftY() {
    return driver.getLeftY();
  }

  public double getLeftX() {
    return driver.getLeftX();
  }

  public double getRightX() {
    return driver.getRightX();
  }

  public double getRightY() {
    return driver.getRightY();
  }

  public void setToNormalDrive() {
    DriveCmds.setDefaultDriveCommand(
        RobotContainer.driveSubsystem, this.normalDriveCmd(), "Default Joystick Drive");
  }

  public Command setToNormalDriveCmd() {
    return DriveCmds.changeDefaultDriveCommand(
        RobotContainer.driveSubsystem, this.normalDriveCmd(), "Default Joystick Drive");
  }

  private Command normalDriveCmd() {
    return DriveCmds.joystickDrive(
        RobotContainer.driveSubsystem,
        () -> -driver.getLeftY(),
        () -> -driver.getLeftX(),
        () -> -driver.getRightX());
  }

  public void setToReefAlign() {
    DriveCmds.setDefaultDriveCommand(
        RobotContainer.driveSubsystem, this.reefAlignCmd(), "Align To Reef");
  }

  public Command setToReefAlignCmd() {
    return DriveCmds.changeDefaultDriveCommand(
        RobotContainer.driveSubsystem, this.reefAlignCmd(), "Align To Reef");
  }

  private Command reefAlignCmd() {
    return DriveCmds.joystickDriveAtAngle(
        RobotContainer.driveSubsystem,
        () -> -driver.getLeftY(),
        () -> -driver.getLeftX(),
        () -> ReefAlign.getInstance().inZone().orElse(RobotContainer.driveSubsystem.getRotation()));
  }

  public void setToSourceAlign() {
    DriveCmds.setDefaultDriveCommand(
        RobotContainer.driveSubsystem, this.sourceAlignCmd(), "Align To Source");
  }

  public Command setToSourceAlignCmd() {
    return DriveCmds.changeDefaultDriveCommand(
        RobotContainer.driveSubsystem, this.sourceAlignCmd(), "Align To Source");
  }

  private Command sourceAlignCmd() {
    return DriveCmds.joystickDriveAtAngle(
        RobotContainer.driveSubsystem,
        () -> -driver.getLeftY(),
        () -> -driver.getLeftX(),
        () ->
            SourceAlign.getInstance().inZone().orElse(RobotContainer.driveSubsystem.getRotation()));
  }
}
