package frc.robot.oi;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.SetpointConstants;
import frc.robot.commands.ClimberCmds;
import frc.robot.commands.DriveCmds;
import frc.robot.commands.RollerCmds;
import frc.robot.commands.climber.Climb;
import frc.robot.commands.scoreassist.ScoreAssistCmds;
import frc.robot.commands.superstructure.EndEffector;
import frc.robot.commands.superstructure.SuperStructure;

public class DriverControls {
  private final CommandXboxController driver = new CommandXboxController(0);

  public void configureButtonBindings() {

    // Reset gyro to 0 deg when start button is pressed
    driver
        .start()
        .onTrue(
            Commands.parallel(
                DriveCmds.changeDefaultDriveCommand(
                    RobotContainer.driveSubsystem,
                    DriveCmds.joystickDrive(
                        RobotContainer.driveSubsystem,
                        () -> -driver.getLeftY(),
                        () -> -driver.getLeftX(),
                        () -> -driver.getRightX()),
                    "Full Control"),
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
                    DriveCmds.changeDefaultDriveCommand(
                        RobotContainer.driveSubsystem,
                        DriveCmds.joystickDrive(
                            RobotContainer.driveSubsystem,
                            () -> -driver.getLeftY(),
                            () -> -driver.getLeftX(),
                            () -> -driver.getRightX()),
                        "Full Control"),
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
        .onTrue(SuperStructure.SOURCE_CORAL_INTAKE.get())
        .onFalse(RollerCmds.setSpeed(() -> 0));

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
                RollerCmds.setSpeed(() -> 0)));

    // Enable/disable reefalign
    driver
        .b()
        .onTrue(Commands.runOnce(() -> RobotContainer.disableReefAlign = false))
        .onFalse(Commands.runOnce(() -> RobotContainer.disableReefAlign = true));

    // Climber
    driver
        .leftTrigger(0.1)
        .whileTrue(new Climb(() -> -1 * driver.getLeftTriggerAxis()))
        .onFalse(ClimberCmds.setVoltage(() -> 0));

    // Grab Algae
    driver
        .rightTrigger(0.2)
        .onTrue(Commands.sequence(EndEffector.CORAL_SCORE.get()))
        .onFalse(SuperStructure.STARTING_CONF.get());

    // POV Precision Driving
    driver
        .povLeft()
        .onTrue(
            DriveCmds.changeDefaultDriveCommand(
                RobotContainer.driveSubsystem,
                DriveCmds.inch(RobotContainer.driveSubsystem, SetpointConstants.Drive.INCH_SPEED),
                "Inch Left"))
        .onFalse(
            DriveCmds.changeDefaultDriveCommand(
                RobotContainer.driveSubsystem,
                DriveCmds.joystickDrive(
                    RobotContainer.driveSubsystem,
                    () -> -driver.getLeftY(),
                    () -> -driver.getLeftX(),
                    () -> -driver.getRightX()),
                "Full Control"));
    driver
        .povRight()
        .onTrue(
            DriveCmds.changeDefaultDriveCommand(
                RobotContainer.driveSubsystem,
                DriveCmds.inch(
                    RobotContainer.driveSubsystem,
                    () -> -1 * SetpointConstants.Drive.INCH_SPEED.getAsDouble()),
                "Inch Right"))
        .onFalse(
            DriveCmds.changeDefaultDriveCommand(
                RobotContainer.driveSubsystem,
                DriveCmds.joystickDrive(
                    RobotContainer.driveSubsystem,
                    () -> -driver.getLeftY(),
                    () -> -driver.getLeftX(),
                    () -> -driver.getRightX()),
                "Full Control"));

    // POV/x heading controller
    // driver
    //     .povLeft()
    //     .onTrue(
    // DriveCmds.changeDefaultDriveCommand(
    //     RobotContainer.driveSubsystem,
    //     DriveCmds.joystickDriveAtAngle(
    //         RobotContainer.driveSubsystem,
    //         () -> -driver.getLeftY(),
    //         () -> -driver.getLeftX(),
    //         () -> Rotation2d.fromDegrees(0)),
    //     "Heading Controller"))
    //     .onFalse(
    //         DriveCmds.changeDefaultDriveCommand(
    //             RobotContainer.driveSubsystem,
    //             DriveCmds.joystickDrive(
    //                 RobotContainer.driveSubsystem,
    //                 () -> -driver.getLeftY(),
    //                 () -> -driver.getLeftX(),
    //                 () -> -driver.getRightX()),
    //             "Full Control"));
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
    // Rotation2d.fromDegrees(RobotContainer.RobotContainer.driveSubsystem.getAngleToReef())),
    //             "Heading Controller"))
    //     .onFalse(
    //         DriveCmds.changeDefaultDriveCommand(
    //             RobotContainer.driveSubsystem,
    //             DriveCmds.joystickDrive(
    //                 RobotContainer.driveSubsystem,
    //                 () -> -driver.getLeftY(),
    //                 () -> -driver.getLeftX(),
    //                 () -> -driver.getRightX()),
    //             "Full Control"));

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
        RobotContainer.driveSubsystem,
        DriveCmds.joystickDrive(
            RobotContainer.driveSubsystem,
            () -> -driver.getLeftY(),
            () -> -driver.getLeftX(),
            () -> -driver.getRightX()),
        "Default Joystick Drive");
  }

  public Command setToNormalDriveCmd() {
    return DriveCmds.changeDefaultDriveCommand(
        RobotContainer.driveSubsystem, this.normalDriveCmd(), "Default Joystick Drive");
  }

  public Command normalDriveCmd() {
    return DriveCmds.joystickDrive(
        RobotContainer.driveSubsystem,
        () -> -driver.getLeftY(),
        () -> -driver.getLeftX(),
        () -> -driver.getRightX());
  }
}
