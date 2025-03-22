package frc.robot.oi;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.SetpointConstants;
import frc.robot.commands.ClimberCmds;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.RollerCmds;
import frc.robot.commands.ScoreAssist;
import frc.robot.commands.climber.Climb;
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
                DriveCommands.changeDefaultDriveCommand(
                    RobotContainer.driveSubsystem,
                    DriveCommands.joystickDrive(
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
                    DriveCommands.changeDefaultDriveCommand(
                        RobotContainer.driveSubsystem,
                        DriveCommands.joystickDrive(
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
        .onTrue(SuperStructure.SOURCE_CORAL_INTAKE)
        .onFalse(RollerCmds.setSpeed(() -> 0));

    // Enable/disable sourcealign
    driver
        .a()
        .onTrue(
            Commands.parallel(
                Commands.runOnce(() -> RobotContainer.disableSourceAlign = false),
                SuperStructure.SOURCE_CORAL_INTAKE))
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

    // Score Coral
    driver
        .rightBumper()
        .onTrue(
            DriveCommands.changeDefaultDriveCommand(
                RobotContainer.driveSubsystem,
                ScoreAssist.getInstance().goReefTracker(RobotContainer.driveSubsystem),
                "ScoreAssist"))
        .onFalse(
            Commands.sequence(
                Commands.runOnce(
                        () -> {
                          ScoreAssist.getInstance().setClosestLocPose(null);
                          ScoreAssist.getInstance().hasStartedCommand = false;
                        })
                    .ignoringDisable(true),
                DriveCommands.changeDefaultDriveCommand(
                    RobotContainer.driveSubsystem,
                    DriveCommands.joystickDrive(
                        RobotContainer.driveSubsystem,
                        () -> -driver.getLeftY(),
                        () -> -driver.getLeftX(),
                        () -> -driver.getRightX()),
                    "Full Control")));

    // Grab Algae
    driver
        .rightTrigger(0.2)
        .onTrue(Commands.sequence(EndEffector.CORAL_SCORE))
        .onFalse(SuperStructure.STARTING_CONF);

    driver
        .povLeft()
        .onTrue(
            DriveCommands.changeDefaultDriveCommand(
                RobotContainer.driveSubsystem,
                DriveCommands.inch(RobotContainer.driveSubsystem, SetpointConstants.Drive.INCH_SPEED),
                "Inch Left"))
        .onFalse(
            DriveCommands.changeDefaultDriveCommand(
                RobotContainer.driveSubsystem,
                DriveCommands.joystickDrive(
                    RobotContainer.driveSubsystem,
                    () -> -driver.getLeftY(),
                    () -> -driver.getLeftX(),
                    () -> -driver.getRightX()),
                "Full Control"));
    driver
        .povRight()
        .onTrue(
            DriveCommands.changeDefaultDriveCommand(
                RobotContainer.driveSubsystem,
                DriveCommands.inch(
                    RobotContainer.driveSubsystem,
                    () -> -1 * SetpointConstants.Drive.INCH_SPEED.getAsDouble()),
                "Inch Right"))

        .onFalse(
            DriveCommands.changeDefaultDriveCommand(
                RobotContainer.driveSubsystem,
                DriveCommands.joystickDrive(
                    RobotContainer.driveSubsystem,
                    () -> -driver.getLeftY(),
                    () -> -driver.getLeftX(),
                    () -> -driver.getRightX()),
                "Full Control"));

    // driver
    //     .povLeft()
    //     .onTrue(
    // DriveCommands.changeDefaultDriveCommand(
    //     RobotContainer.driveSubsystem,
    //     DriveCommands.joystickDriveAtAngle(
    //         RobotContainer.driveSubsystem,
    //         () -> -driver.getLeftY(),
    //         () -> -driver.getLeftX(),
    //         () -> Rotation2d.fromDegrees(0)),
    //     "Heading Controller"))
    //     .onFalse(
    //         DriveCommands.changeDefaultDriveCommand(
    //             RobotContainer.driveSubsystem,
    //             DriveCommands.joystickDrive(
    //                 RobotContainer.driveSubsystem,
    //                 () -> -driver.getLeftY(),
    //                 () -> -driver.getLeftX(),
    //                 () -> -driver.getRightX()),
    //             "Full Control"));
    // driver
    //     .x()
    //     .onTrue(
    //         DriveCommands.changeDefaultDriveCommand(
    //             RobotContainer.driveSubsystem,
    //             DriveCommands.joystickDriveAtAngle(
    //                 RobotContainer.driveSubsystem,
    //                 () -> -driver.getLeftY(),
    //                 () -> -driver.getLeftX(),
    //                 () ->
    // Rotation2d.fromDegrees(RobotContainer.RobotContainer.driveSubsystem.getAngleToReef())),
    //             "Heading Controller"))
    //     .onFalse(
    //         DriveCommands.changeDefaultDriveCommand(
    //             RobotContainer.driveSubsystem,
    //             DriveCommands.joystickDrive(
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

  public void setToNormalDrive() {
    DriveCommands.setDefaultDriveCommand(
        RobotContainer.driveSubsystem,
        DriveCommands.joystickDrive(
            RobotContainer.driveSubsystem,
            () -> -driver.getLeftY(),
            () -> -driver.getLeftX(),
            () -> -driver.getRightX()),
        "Default Joystick Drive");
  }
}
