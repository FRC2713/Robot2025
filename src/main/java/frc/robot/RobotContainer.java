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
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveConstants.OTFConstants;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ScoreAssist;
import frc.robot.commands.autos.AutoRoutines;
import frc.robot.commands.superstructure.ElevatorCmds;
import frc.robot.commands.superstructure.PivotCmds;
import frc.robot.commands.superstructure.RollerCmds;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drivetrain;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.pivot.PivotIO;
import frc.robot.subsystems.pivot.PivotSparks;
import frc.robot.subsystems.rollers.Rollers;
import frc.robot.subsystems.rollers.Rollers1xSim;
import frc.robot.subsystems.rollers.Rollers1xSpark;
import frc.robot.subsystems.rollers.RollersIO;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.AllianceFlipUtil;
import java.util.Arrays;
import org.littletonrobotics.junction.Logger;

public class RobotContainer {
  // Subsystems
  public static Drivetrain driveSubsystem;
  public static Pivot pivotThing;
  public static Elevator elevator;
  public static Rollers rollers;
  // Xbox Controllers
  private final CommandXboxController driver = new CommandXboxController(0);

  // Dashboard inputs
  public final AutoChooser autoChooser;

  // For Choreo
  private final AutoFactory choreoAutoFactory;
  private final SparkMax outtake = new SparkMax(50, MotorType.kBrushless);
  private final Vision visionsubsystem;

  public RobotContainer() {
    // Start subsystems
    visionsubsystem = new Vision();
    switch (Constants.currentMode) {
      case REAL:
        driveSubsystem =
            new Drivetrain(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        pivotThing = new Pivot(new PivotSparks());
        rollers = new Rollers(new Rollers1xSpark());
        break;

      case SIM:
        driveSubsystem =
            new Drivetrain(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        pivotThing = new Pivot(new PivotIO() {}); // TODO: implement sim
        elevator = new Elevator(new ElevatorIO() {}); // TODO: implement sim
        rollers = new Rollers(new Rollers1xSim());
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
        pivotThing = new Pivot(new PivotIO() {});
        rollers = new Rollers(new RollersIO() {});
        break;
    }

    // PathPlanner Config
    AutoBuilder.configure(
        driveSubsystem::getPose, // Robot pose supplier
        driveSubsystem
            ::resetOdometry, // Method to reset odometry (will be called if your auto has a starting
        // pose)
        driveSubsystem::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        (speeds, feedforwards) ->
            driveSubsystem.runVelocity(
                speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds.
        // Also optionally outputs individual module feedforwards
        new PPHolonomicDriveController( // PPHolonomicController is the built in path following
            // controller for holonomic drive trains
            OTFConstants.translationPID, // Translation PID constants
            OTFConstants.rotationPID // Rotation PID constants
            ),
        DriveConstants.pathPlannerConfig, // The robot configuration
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        driveSubsystem // Reference to this subsystem to set requirements
        );
    ScoreAssist.initCommands();

    // Choreo Autos
    choreoAutoFactory =
        new AutoFactory(
            driveSubsystem::getPose,
            driveSubsystem::resetOdometry,
            driveSubsystem::followTrajectory,
            true,
            driveSubsystem,
            (sample, isStart) -> {
              Logger.recordOutput(
                  "ActiveTrajectory",
                  Arrays.stream(sample.getPoses())
                      .map(AllianceFlipUtil::apply)
                      .toArray(Pose2d[]::new));
            });

    AutoRoutines autoRoutines = new AutoRoutines(choreoAutoFactory);

    autoChooser = new AutoChooser();

    // Add options to the chooser
    autoChooser.addRoutine("Example Auto Command", autoRoutines::exampleAuto);
    autoChooser.addRoutine(
        "DriveStraight",
        () -> {
          AutoRoutine routine = choreoAutoFactory.newRoutine("DriveStraight");

          var startToReefTraj = routine.trajectory("Example");

          // When the routine begins, reset odometry and start the first trajectory
          routine
              .active()
              .onTrue(
                  Commands.sequence(
                      new InstantCommand(() -> System.out.println("DriveStraight started")),
                      startToReefTraj.resetOdometry(),
                      startToReefTraj.cmd()));

          return routine;
        });

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
    DriveCommands.setDefaultDriveCommand(
        driveSubsystem,
        DriveCommands.joystickDrive(
            driveSubsystem,
            () -> -driver.getLeftY(),
            () -> -driver.getLeftX(),
            () -> -driver.getRightX()),
        "Full Control");

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

    // Reset gyro to 180° when start button is pressed
    driver
        .back()
        .onTrue(
            Commands.runOnce(
                    () ->
                        driveSubsystem.setPose(
                            new Pose2d(
                                driveSubsystem.getPose().getTranslation(),
                                Rotation2d.fromDegrees(180))),
                    driveSubsystem)
                .ignoringDisable(true));

    driver
        .rightBumper()
        .onTrue(Commands.sequence(new InstantCommand(() -> outtake.setVoltage(-2.5))))
        .toggleOnFalse(new InstantCommand(() -> outtake.setVoltage(0)));

    driver.a().whileTrue(ScoreAssist.scoreClosestL1(driveSubsystem));

    driver
        .x()
        .onTrue(
            Commands.parallel(
                ElevatorCmds.setHeight(60),
                PivotCmds.setAngle(60),
                RollerCmds.setAlgaeSpeed(4000)));
    driver
        .y()
        .onTrue(
            Commands.parallel(
                ElevatorCmds.setHeight(0), PivotCmds.setAngle(0), RollerCmds.setTubeSpeed(4000)));

    // Heading controller
    driver
        .povUp()
        .onTrue(
            DriveCommands.changeDefaultDriveCommand(
                driveSubsystem,
                DriveCommands.joystickDriveAtAngle(
                    driveSubsystem,
                    () -> -driver.getLeftY(),
                    () -> -driver.getLeftX(),
                    () -> Rotation2d.fromDegrees(180)),
                "Heading Controller"))
        .onFalse(
            DriveCommands.changeDefaultDriveCommand(
                driveSubsystem,
                DriveCommands.joystickDrive(
                    driveSubsystem,
                    () -> -driver.getLeftY(),
                    () -> -driver.getLeftX(),
                    () -> -driver.getRightX()),
                "Full Control"));

    driver
        .povDown()
        .onTrue(
            DriveCommands.changeDefaultDriveCommand(
                driveSubsystem,
                DriveCommands.joystickDriveAtAngle(
                    driveSubsystem,
                    () -> -driver.getLeftY(),
                    () -> -driver.getLeftX(),
                    () -> Rotation2d.fromDegrees(0)),
                "Heading Controller"))
        .onFalse(
            DriveCommands.changeDefaultDriveCommand(
                driveSubsystem,
                DriveCommands.joystickDrive(
                    driveSubsystem,
                    () -> -driver.getLeftY(),
                    () -> -driver.getLeftX(),
                    () -> -driver.getRightX()),
                "Full Control"));

    driver
        .povLeft()
        .onTrue(
            DriveCommands.changeDefaultDriveCommand(
                driveSubsystem,
                DriveCommands.joystickDriveAtAngle(
                    driveSubsystem,
                    () -> -driver.getLeftY(),
                    () -> -driver.getLeftX(),
                    () -> Rotation2d.fromDegrees(-90)),
                "Heading Controller"))
        .onFalse(
            DriveCommands.changeDefaultDriveCommand(
                driveSubsystem,
                DriveCommands.joystickDrive(
                    driveSubsystem,
                    () -> -driver.getLeftY(),
                    () -> -driver.getLeftX(),
                    () -> -driver.getRightX()),
                "Full Control"));

    driver
        .povRight()
        .onTrue(
            DriveCommands.changeDefaultDriveCommand(
                driveSubsystem,
                DriveCommands.joystickDriveAtAngle(
                    driveSubsystem,
                    () -> -driver.getLeftY(),
                    () -> -driver.getLeftX(),
                    () -> Rotation2d.fromDegrees(90)),
                "Heading Controller"))
        .onFalse(
            DriveCommands.changeDefaultDriveCommand(
                driveSubsystem,
                DriveCommands.joystickDrive(
                    driveSubsystem,
                    () -> -driver.getLeftY(),
                    () -> -driver.getLeftX(),
                    () -> -driver.getRightX()),
                "Full Control"));
  }
}
