// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoFactory.AutoBindings;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.FullAutoRoutines;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drivetrain;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;

public class RobotContainer {
  // Subsystems
  private final Drivetrain driveSubsystem;

  // Xbox Controllers
  private final CommandXboxController driver = new CommandXboxController(0);

  // Dashboard inputs
  // private final LoggedDashboardChooser<Command> autoChooser;

  // For Choreo
  private final AutoFactory choreoAutoFactory;

  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        driveSubsystem =
            new Drivetrain(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        break;

      case SIM:
        driveSubsystem =
            new Drivetrain(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        break;

      default:
        // Replayed robot, disable IO implementations
        driveSubsystem =
            new Drivetrain(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        break;
    }

    choreoAutoFactory = new AutoFactory(
      driveSubsystem::getPose,
      driveSubsystem::resetOdometry,
      driveSubsystem::followTrajectory, 
      true, 
      driveSubsystem,
      new AutoBindings(),
      null);

    // Set up auto routines
    // autoChooser = new LoggedDashboardChooser<>("Auto Choices");

    // autoChooser.addOption("Test Auto", FullAutoRoutines.simplePathAuto(driveSubsystem, choreoAutoFactory));
  
    // Manual Sysid of drive train using AdvantageKit
    // autoChooser.addOption(
    //     "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(driveSubsystem));
    // autoChooser.addOption(
    //     "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(driveSubsystem));
    // autoChooser.addOption(
    //     "Drive SysId (Quasistatic Forward)",
    //     driveSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Drive SysId (Quasistatic Reverse)",
    //     driveSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // autoChooser.addOption(
    //     "Drive SysId (Dynamic Forward)", driveSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Drive SysId (Dynamic Reverse)", driveSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    driveSubsystem.setDefaultCommand(
        DriveCommands.joystickDrive(
            driveSubsystem,
            () -> -driver.getLeftY(),
            () -> -driver.getLeftX(),
            () -> -driver.getRightX()));

    // Reset gyro to 0° when start button is pressed
    driver
        .start()
        .onTrue(
            Commands.runOnce(
                    () ->
                        driveSubsystem.setPose(
                            new Pose2d(driveSubsystem.getPose().getTranslation(), new Rotation2d())),
                    driveSubsystem)
                .ignoringDisable(true));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new InstantCommand(() -> System.out.println("hi"));
  }
}
