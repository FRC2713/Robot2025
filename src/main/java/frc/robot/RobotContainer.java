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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.commands.AlgaeClawCmds;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.RollerCmds;
import frc.robot.commands.ScoreAssist;
import frc.robot.commands.SuperStructure;
import frc.robot.commands.autos.AutoRoutines;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.algaeClaw.AlgaeClaw;
import frc.robot.subsystems.algaeClaw.AlgaeClawIO;
import frc.robot.subsystems.algaeClaw.AlgaeClawIOSim;
import frc.robot.subsystems.algaeClaw.AlgaeClawIOSparks;
import frc.robot.subsystems.constants.DriveConstants;
import frc.robot.subsystems.constants.DriveConstants.OTFConstants;
import frc.robot.subsystems.constants.VisionConstants;
import frc.robot.subsystems.drive.Drivetrain;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOKrakens;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.pivot.PivotIO;
import frc.robot.subsystems.pivot.PivotIOKrakens;
import frc.robot.subsystems.pivot.PivotIOSim;
import frc.robot.subsystems.rollers.Rollers;
import frc.robot.subsystems.rollers.RollersIO;
import frc.robot.subsystems.rollers.RollersIOSim;
import frc.robot.subsystems.rollers.RollersIOSparks;
import frc.robot.subsystems.shoulder.Shoulder;
import frc.robot.subsystems.shoulder.ShoulderIO;
import frc.robot.subsystems.shoulder.ShoulderIOKrakens;
import frc.robot.subsystems.shoulder.ShoulderIOSim;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOOdometry;
import frc.robot.subsystems.vision.VisionIOPoseEstimator;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.RHRHolonomicDriveController;
import java.util.Arrays;
import org.littletonrobotics.junction.Logger;

public class RobotContainer {
  // Subsystems
  public static Drivetrain driveSubsystem;
  public static Elevator elevator;
  public static Shoulder shoulder;
  public static Pivot pivot;
  public static Rollers rollers;
  public static AlgaeClaw algaeClaw;
  // Xbox Controllers
  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);

  // Dashboard inputs
  public final AutoChooser autoChooser;

  public static final RHRHolonomicDriveController otfController =
      new RHRHolonomicDriveController(
          OTFConstants.translationPID, // Translation PID constants
          OTFConstants.rotationPID, // Rotation PID constants
          OTFConstants.translationTolerance // Translation Tolerenace
          );

  // For Choreo
  private final AutoFactory choreoAutoFactory;
  public static Vision visionsubsystem;

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

        elevator = new Elevator(new ElevatorIOKrakens());
        pivot = new Pivot(new PivotIOKrakens());
        rollers = new Rollers(new RollersIOSparks());
        algaeClaw = new AlgaeClaw(new AlgaeClawIOSparks());
        shoulder = new Shoulder(new ShoulderIOKrakens());
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
        algaeClaw = new AlgaeClaw(new AlgaeClawIOSim());
        shoulder = new Shoulder(new ShoulderIOSim());
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
        algaeClaw = new AlgaeClaw(new AlgaeClawIO() {});
        shoulder = new Shoulder(new ShoulderIO() {});
        break;
    }
    visionsubsystem =
        new Vision(
            VisionConstants.USE_WHEEL_ODOMETRY
                ? new VisionIOOdometry()
                : new VisionIOPoseEstimator());

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
        RobotContainer.otfController,
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
    autoChooser.addRoutine("Coral and Algae Auto", autoRoutines::coralAndAlgaeAuto);
    autoChooser.addRoutine("Score Lots Of Coral", autoRoutines::scoreLotsOfCoral);
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

    // Uncomment for swerve drive characterization
    // autoChooser.addCmd(
    //     "Drive Simple FF Characterization",
    //     () -> DriveCommands.feedforwardCharacterization(driveSubsystem));
    // autoChooser.addCmd(
    //     "Drive SysId (Quasistatic Forward)",
    //     () -> driveSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // autoChooser.addCmd(
    //     "Drive SysId (Quasistatic Backward)",
    //     () -> driveSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // autoChooser.addCmd(
    //     "Drive SysId (Dynamic Forward)",
    //     () -> driveSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // autoChooser.addCmd(
    //     "Drive SysId (Dynamic Backward)",
    //     () -> driveSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    // autoChooser.addCmd(
    //     "Wheel Radius", () -> DriveCommands.wheelRadiusCharacterization(driveSubsystem));

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
        "Default Joystick Drive");

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

    // Intake Coral
    driver
        .leftBumper()
        .whileTrue(SuperStructure.SOURCE_CORAL_INTAKE.getCommand())
        .onFalse(SuperStructure.STARTING_CONF.getCommand());

    // Score Coral
    driver.leftTrigger(0.25).whileTrue(RollerCmds.score(SSConstants.Roller.L3_CORAL_SCORE_SPEED));

    // Intake Algae
    driver
        .rightBumper()
        .whileTrue(AlgaeClawCmds.intake(SSConstants.AlgaeClaw.L3_ALGAE_GRAB_SPEED))
        .onFalse(RollerCmds.setSpeed(() -> 0));

    // Score Algae
    driver
        .rightTrigger(0.25)
        .onTrue(AlgaeClawCmds.score(SSConstants.AlgaeClaw.PROCESSOR_SCORE_SPEED))
        .onFalse(AlgaeClawCmds.setSpeed(() -> 0));

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

    operator.a().onTrue(SuperStructure.L1_CORAL_PREP.getCommand());
    operator.b().onTrue(SuperStructure.L2_CORAL_PREP.getCommand());
    operator.x().onTrue(SuperStructure.L3_CORAL_PREP.getCommand());

    operator.povDown().onTrue(SuperStructure.PROCESSOR_SCORE.getCommand());
    operator.povRight().onTrue(SuperStructure.L1_ALGAE_GRAB.getCommand());
    operator.povLeft().onTrue(SuperStructure.L3_ALGAE_GRAB.getCommand());

    operator.leftBumper().onTrue(SuperStructure.STARTING_CONF.getCommand());

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
    rollers.setRPM(0);
  }
}
