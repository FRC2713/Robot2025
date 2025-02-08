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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.PivotCmds;
import frc.robot.commands.RollerCmds;
import frc.robot.commands.ScoreAssist;
import frc.robot.commands.SuperStructure;
import frc.robot.commands.autos.AutoRoutines;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.constants.DriveConstants;
import frc.robot.subsystems.constants.DriveConstants.OTFConstants;
import frc.robot.subsystems.drive.Drivetrain;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.pivot.PivotIO;
import frc.robot.subsystems.pivot.PivotIOKrakens;
import frc.robot.subsystems.pivot.PivotIOSim;
import frc.robot.subsystems.rollers.Rollers;
import frc.robot.subsystems.rollers.RollersIO;
import frc.robot.subsystems.rollers.RollersIOSim;
import frc.robot.subsystems.rollers.RollersIOSparks;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOOdometry;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.ScoreLevel;
import java.util.Arrays;
import org.littletonrobotics.junction.Logger;

public class RobotContainer {
  // Subsystems
  public static Drivetrain driveSubsystem;
  public static Elevator elevator;
  public static Pivot pivot;
  public static Rollers rollers;
  // Xbox Controllers
  private final CommandXboxController driver = new CommandXboxController(0);

  // Dashboard inputs
  public final AutoChooser autoChooser;

  // For Choreo
  private final AutoFactory choreoAutoFactory;
  private static Vision visionsubsystem;

  public RobotContainer() {
    // Start subsystems
    switch (Constants.currentMode) {
      case REAL:
        driveSubsystem =
            new Drivetrain(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        elevator = new Elevator(new ElevatorIO() {});
        pivot = new Pivot(new PivotIOKrakens());
        rollers = new Rollers(new RollersIOSparks() {});
        break;

      case SIM:
        driveSubsystem =
            new Drivetrain(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        pivot = new Pivot(new PivotIOSim());
        elevator = new Elevator(new ElevatorIOSim());
        rollers = new Rollers(new RollersIOSim());
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
        elevator = new Elevator(new ElevatorIO() {});
        pivot = new Pivot(new PivotIO() {});
        rollers = new Rollers(new RollersIO() {});
        break;
    }
    visionsubsystem = new Vision(new VisionIOOdometry());

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

    autoChooser.addRoutine("ScoreLotsOfCoral", autoRoutines::scoreLotsOfCoral);

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
        "Full Control Modified");

    // Reset gyro to 0 deg when start button is pressed
    driver
        .start()
        .onTrue(
            Commands.runOnce(
                    () ->
                        driveSubsystem.setPose(
                            new Pose2d(
                                driveSubsystem.getPose().getTranslation(),
                                Rotation2d.fromDegrees(0))),
                    driveSubsystem)
                .ignoringDisable(true));

    // Reset gyro to 180 deg when start button is pressed
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
        .a()
        .onTrue(
            ScoreAssist.getInstance()
                .setActiveCommand(
                    () -> ScoreAssist.getClosestCommand(driveSubsystem::getPose, ScoreLevel.ONE)))
        .onFalse(
            Commands.sequence(
                ScoreAssist.getInstance().cancelCmd(), SuperStructure.STARTING_CONF.getCommand()));

    driver
        .b()
        .onTrue(
            ScoreAssist.getInstance()
                .setActiveCommand(
                    () -> ScoreAssist.getClosestCommand(driveSubsystem::getPose, ScoreLevel.TWO)))
        .onFalse(
            Commands.sequence(
                ScoreAssist.getInstance().cancelCmd(), SuperStructure.STARTING_CONF.getCommand()));

    driver
        .leftBumper()
        .whileTrue(Commands.sequence(PivotCmds.setAngle(55), RollerCmds.setTubeSpeed(() -> 1000)))
        .onFalse(Commands.sequence(PivotCmds.setAngle(5), RollerCmds.setTubeSpeed(() -> 0)));

    driver
        .rightBumper()
        .whileTrue(SuperStructure.L1_CORAL_SCORE.getCommand())
        .onFalse(SuperStructure.STARTING_CONF.getCommand());

    // Slow-Mode
    driver
        .rightBumper()
        .onTrue(
            DriveCommands.changeDefaultDriveCommand(
                driveSubsystem,
                DriveCommands.joystickDrive(
                    driveSubsystem,
                    () -> -driver.getLeftY() * 0.3,
                    () -> -driver.getLeftX() * 0.3,
                    () -> -driver.getRightX() * 0.3),
                "Slow-Mode"))
        .onFalse(
            DriveCommands.changeDefaultDriveCommand(
                driveSubsystem,
                DriveCommands.joystickDrive(
                    driveSubsystem,
                    () -> -driver.getLeftY(),
                    () -> -driver.getLeftX(),
                    () -> -driver.getRightX()),
                "Full Control"));

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

    ScoreAssist.getInstance()
        .getTrigger()
        .onTrue(
            ScoreAssist.getInstance()
                .setActiveCommand(ScoreAssist.getInstance()::networkTablesDrive))
        .onFalse(ScoreAssist.getInstance().cancelCmd());
  }

  public void disabledPeriodic() {
    // Safety
    elevator.setTargetHeight(elevator.getCurrentHeight());
    pivot.setTargetAngle(pivot.getCurrentAngle());
    rollers.setTubeRPM(0);
  }
}
