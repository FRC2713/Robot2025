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
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.SuperStructure;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drivetrain;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;

public class RobotContainer {
  // Subsystems
  private final Drivetrain driveSubsystem;

  // Xbox Controllers
  private final CommandXboxController driver = new CommandXboxController(0);

  // Dashboard inputs
  public final AutoChooser autoChooser;

  // For Choreo
  private final AutoFactory choreoAutoFactory;

  private final SparkMax outtake = new SparkMax(50, MotorType.kBrushless);

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

    choreoAutoFactory =
        new AutoFactory(
            driveSubsystem::getPose,
            driveSubsystem::resetOdometry,
            driveSubsystem::followTrajectory,
            true,
            driveSubsystem);

    autoChooser = new AutoChooser();

    // Add options to the chooser
    autoChooser.addRoutine("Example Auto Command", this::exampleAuto);

    autoChooser.addCmd(
        "Drive Simple FF Characterization",
        () -> DriveCommands.feedforwardCharacterization(driveSubsystem));
    autoChooser.addCmd(
        "Drive SysId (Quasistatic Forward)",
        () -> driveSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addCmd(
        "Drive SysId (Quasistatic Backward)",
        () -> driveSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addCmd(
        "Drive SysId (Dynamic Forward)",
        () -> driveSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addCmd(
        "Drive SysId (Dynamic Backward)",
        () -> driveSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    autoChooser.addCmd(
        "Wheel Radius", () -> DriveCommands.wheelRadiusCharacterization(driveSubsystem));

    // Put the auto chooser on the dashboard
    SmartDashboard.putData("Auto Chooser", autoChooser);

    // Schedule the auto
    RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());

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
                            new Pose2d(
                                driveSubsystem.getPose().getTranslation(), new Rotation2d())),
                    driveSubsystem)
                .ignoringDisable(true));

    driver
        .rightBumper()
        .onTrue(Commands.sequence(new InstantCommand(() -> outtake.setVoltage(-5))))
        .toggleOnFalse(new InstantCommand(() -> outtake.setVoltage(0)));
  }

  public AutoRoutine exampleAuto() {
    AutoRoutine routine = choreoAutoFactory.newRoutine("Example Auto");

    // Load the routine's trajectories
    AutoTrajectory startToReefTraj = routine.trajectory("StartToReef");
    AutoTrajectory reefToProcTraj = routine.trajectory("ReefToProcessor");

    // When the routine begins, reset odometry and start the first trajectory
    routine
        .active()
        .onTrue(
            Commands.sequence(
                new InstantCommand(() -> System.out.println("Example Auto started")),
                startToReefTraj.resetOdometry(),
                startToReefTraj.cmd()));

    // Starting at the event marker named "intake", run the intake
    startToReefTraj.atTime("StartElevator").onTrue(SuperStructure.L1_CORAL_PREP_ELEVATOR());

    // // When the trajectory is done, start the next trajectory
    startToReefTraj
        .done()
        .onTrue(
            Commands.sequence(
                SuperStructure.L1_CORAL_SCORE_AND_ALGAE_TAKE(), reefToProcTraj.cmd()));

    // // While the trajectory is active, prepare the scoring subsystem
    reefToProcTraj.active().whileTrue(SuperStructure.PROCESSOR_PREP());

    // // When the trajectory is done, score
    reefToProcTraj.done().onTrue(SuperStructure.PROCESSOR_SCORE());

    return routine;
  }
}
