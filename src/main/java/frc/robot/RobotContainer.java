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
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ClimberCmds;
import frc.robot.commands.DriveCmds;
import frc.robot.commands.ElevatorCmds;
import frc.robot.commands.RollerCmds;
import frc.robot.commands.SuperStructure;
import frc.robot.commands.autos.CenterAutoPineTree;
import frc.robot.commands.autos.CoralAndAlgaeAuto;
import frc.robot.commands.autos.DriveTesting;
import frc.robot.commands.autos.ScoreLotsOfCoral;
import frc.robot.commands.autos.ScoreLotsOfCoralFlipped;
import frc.robot.commands.scoreassist.ScoreAssistCmds;
import frc.robot.generated.TunerConstants;
import frc.robot.scoreassist.ReefAlign;
import frc.robot.scoreassist.ScoreAssist;
import frc.robot.scoreassist.SourceAlign;
import frc.robot.subsystems.algaeClaw.AlgaeClaw;
import frc.robot.subsystems.algaeClaw.AlgaeClawIO;
import frc.robot.subsystems.algaeClaw.AlgaeClawIOSim;
import frc.robot.subsystems.algaeClaw.AlgaeClawIOSparks;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOSparks;
import frc.robot.subsystems.constants.DriveConstants;
import frc.robot.subsystems.constants.DriveConstants.OTFConstants;
import frc.robot.subsystems.constants.VisionConstants;
import frc.robot.subsystems.constants.VisionConstants.VisionOptions;
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
import frc.robot.subsystems.vision.VisionIOLimelights;
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
  public static Climber climber;

  // Xbox Controllers
  public static final CommandXboxController driver = new CommandXboxController(0);
  public static final CommandXboxController operator = new CommandXboxController(1);

  // Dashboard inputs
  public final AutoChooser autoChooser;
  private static boolean hasRanAuto = false;

  public static final RHRHolonomicDriveController otfController =
      new RHRHolonomicDriveController(
          OTFConstants.translationPID, // Translation PID constants
          OTFConstants.rotationPID, // Rotation PID constants
          OTFConstants.translationTolerance // Translation Tolerenace
          );

  // For Choreo
  private final AutoFactory choreoAutoFactory;

  // For teleop automation
  public static ScoreAssist scoreAssist;
  public static Vision visionsubsystem;
  public static boolean disableReefAlign = false;
  public static boolean disableSourceAlign = true;
  private Trigger reefAlignTrigger = new Trigger(ReefAlign.getInstance()::shouldDoReefAlign);
  private Trigger sourceAlignTrigger = new Trigger(SourceAlign.getInstance()::shouldDoSourceAlign);
  // private Trigger climbPrepTrigger = new
  // Trigger(ScoreAssistOld.getInstance()::shouldClimbPrep);

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
        climber = new Climber(new ClimberIOSparks());
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
        climber = new Climber(new ClimberIO() {});
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
        climber = new Climber(new ClimberIO() {});
        break;
    }
    visionsubsystem =
        new Vision(
            // new VisionIO() {}
            VisionConstants.ACTIVE_VISION_OPTION == VisionOptions.SLAMDUNK_WHEEL_ODOMETRY
                ? new VisionIOOdometry()
                : (VisionConstants.ACTIVE_VISION_OPTION == VisionOptions.SLAMDUNK
                    ? new VisionIOPoseEstimator()
                    : new VisionIOLimelights(
                        VisionConstants.FRONT_LIMELIGHT_INFO,
                        VisionConstants.BACK_LIMELIGHT_INFO,
                        driveSubsystem)));

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

    autoChooser = new AutoChooser();

    // Add options to the chooser
    // I add a * to the name when it generates its starting trajectory
    autoChooser.addRoutine(
        "Center Pine Tree", () -> CenterAutoPineTree.getRoutine(choreoAutoFactory, driveSubsystem));
    autoChooser.addRoutine(
        "Score Lots Of Coral",
        () -> ScoreLotsOfCoral.getRoutine(choreoAutoFactory, driveSubsystem));
    autoChooser.addRoutine(
        "Score Lots Of Coral Flipped",
        () -> ScoreLotsOfCoralFlipped.getRoutine(choreoAutoFactory, driveSubsystem));
    autoChooser.addRoutine(
        "Coral and Algae Auto",
        () -> CoralAndAlgaeAuto.getRoutine(choreoAutoFactory, driveSubsystem));
    autoChooser.addRoutine(
        "Drive Testing", () -> DriveTesting.getRoutine(choreoAutoFactory, driveSubsystem));

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
    autoChooser.addCmd("Wheel Radius", () -> DriveCmds.wheelRadiusCharacterization(driveSubsystem));
    // Put the auto chooser on the dashboard
    SmartDashboard.putData("Auto Chooser", autoChooser);

    // Schedule the auto
    RobotModeTriggers.autonomous()
        .whileTrue(
            Commands.sequence(
                new InstantCommand(() -> RobotContainer.hasRanAuto = true),
                autoChooser.selectedCommandScheduler()));

    scoreAssist = new ScoreAssist();

    // Configure the button bindings
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    setToNormalDrive();

    reefAlignTrigger
        .onTrue(
            DriveCmds.changeDefaultDriveCommand(
                driveSubsystem,
                DriveCmds.joystickDriveAtAngle(
                    driveSubsystem,
                    () -> -driver.getLeftY(),
                    () -> -driver.getLeftX(),
                    () -> ReefAlign.getInstance().inZone().orElse(driveSubsystem.getRotation())),
                "Align To Reef"))
        .onFalse(setToNormalDriveCmd());

    sourceAlignTrigger
        .onTrue(
            DriveCmds.changeDefaultDriveCommand(
                driveSubsystem,
                DriveCmds.joystickDriveAtAngle(
                    driveSubsystem,
                    () -> -driver.getLeftY(),
                    () -> -driver.getLeftX(),
                    () -> SourceAlign.getInstance().inZone().orElse(driveSubsystem.getRotation())),
                "Align To Source"))
        .onFalse(setToNormalDriveCmd());

    // Reset gyro to 0 deg when start button is pressed
    driver
        .start()
        .onTrue(
            Commands.parallel(
                setToNormalDriveCmd(),
                Commands.runOnce(
                        () ->
                            driveSubsystem.setPose(
                                new Pose2d(
                                    driveSubsystem.getPose().getTranslation(),
                                    Rotation2d.fromDegrees(0))),
                        driveSubsystem)
                    .ignoringDisable(true)));

    // Reset gyro to 180 deg when start button is pressed
    driver
        .back()
        .onTrue(
            Commands.parallel(
                    setToNormalDriveCmd(),
                    Commands.runOnce(
                        () ->
                            driveSubsystem.setPose(
                                new Pose2d(
                                    driveSubsystem.getPose().getTranslation(),
                                    Rotation2d.fromDegrees(180))),
                        driveSubsystem))
                .ignoringDisable(true));

    // Intake Coral
    driver
        .leftBumper()
        // .onTrue(ScoreAssistCmds.intake())
        .whileTrue(SuperStructure.SOURCE_CORAL_INTAKE.getCommand())
        // .onFalse(Commands.sequence(ScoreAssistCmds.stop()));
        .onFalse(RollerCmds.setSpeed(() -> 0));

    // Score Assist
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
                SuperStructure.SOURCE_CORAL_INTAKE.getCommand()))
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
        .whileTrue(
            Commands.sequence(
                Commands.either(
                    ClimberCmds.configureSoftLimits(
                        SSConstants.Climber.MIN_ANGLE_CLIMBING,
                        SSConstants.Climber.MAX_ANGLE_CLIMBING),
                    Commands.none(),
                    () -> climber.getCurrentAngle() > 100),
                ClimberCmds.moveClimber(
                    () -> -1 * driver.getLeftTriggerAxis(), SSConstants.Climber.SERVO_POS_ON)))
        .onFalse(ClimberCmds.setVoltage(() -> 0));

    // Grab Algae
    driver
        .rightTrigger(0.2)
        .onTrue(Commands.sequence(SuperStructure.CORAL_SCORE.getCommand()))
        .onFalse(SuperStructure.STARTING_CONF.getCommand());

    // // Score Algae
    // // just spit the algae, it's up to the operator to put it in processor, intake, or barge pose
    // driver
    //     .rightTrigger(0.25)
    //     .onTrue(AlgaeClawCmds.setSpeed(SSConstants.AlgaeClaw.PROCESSOR_SCORE_SPEED))
    //     .onFalse(AlgaeClawCmds.setSpeed(() -> 0));

    // Uncomment to use POV buttons for heading controller testing
    // driver
    //     .x()
    //     .onTrue(
    //         DriveCommands.changeDefaultDriveCommand(
    //             driveSubsystem,
    //             DriveCommands.joystickDriveAtAngle(
    //                 driveSubsystem,
    //                 () -> -driver.getLeftY(),
    //                 () -> -driver.getLeftX(),
    //                 () -> Rotation2d.fromDegrees(180)),
    //             "Heading Controller"))
    //     .onFalse(normalDriveCmd());
    // driver
    //     .a()
    //     .onTrue(
    //         DriveCommands.changeDefaultDriveCommand(
    //             driveSubsystem,
    //             DriveCommands.joystickDriveAtAngle(
    //                 driveSubsystem,
    //                 () -> -driver.getLeftY(),
    //                 () -> -driver.getLeftX(),
    //                 () -> Rotation2d.fromDegrees(0)),
    //             "Heading Controller"))
    //     .onFalse(normalDriveCmd());

    // Uncomment to use POV buttons for slow mode
    driver
        .povLeft()
        .onTrue(
            DriveCmds.changeDefaultDriveCommand(
                driveSubsystem,
                DriveCmds.inch(driveSubsystem, SSConstants.Drive.INCH_SPEED),
                "Inch Left"))
        .onFalse(setToNormalDriveCmd());
    driver
        .povRight()
        .onTrue(
            DriveCmds.changeDefaultDriveCommand(
                driveSubsystem,
                DriveCmds.inch(
                    driveSubsystem, () -> -1 * SSConstants.Drive.INCH_SPEED.getAsDouble()),
                "Inch Right"))
        .onFalse(setToNormalDriveCmd());

    operator.povUp().onTrue(ElevatorCmds.setHeight(20));
    operator.povDown().onTrue(ElevatorCmds.setHeight(5));

    // Operator Controls
    operator.a().onTrue(SuperStructure.L1.getCommand());
    operator.b().onTrue(SuperStructure.L2.getCommand());
    operator.y().onTrue(SuperStructure.L3.getCommand());
    operator.rightBumper().onTrue(SuperStructure.L4.getCommand());

    // climbPrepTrigger.onTrue(
    //     Commands.parallel(
    //         DriveCommands.changeDefaultDriveCommand(
    //             driveSubsystem,
    //             DriveCommands.joystickDriveSlow(
    //                 driveSubsystem,
    //                 () -> -driver.getLeftY(),
    //                 () -> -driver.getLeftX(),
    //                 () -> -driver.getRightX()),
    //             "Slow Control"),
    //         SuperStructure.CLIMB_PREP.getCommand()));

    operator
        .start()
        .onTrue(
            // Set driving to slow mode first
            Commands.parallel(
                DriveCmds.changeDefaultDriveCommand(
                    driveSubsystem,
                    DriveCmds.joystickDriveSlow(
                        driveSubsystem,
                        () -> -driver.getLeftY(),
                        () -> -driver.getLeftX(),
                        () -> -driver.getRightX()),
                    "Slow Control"),
                SuperStructure.CLIMB_PREP.getCommand()));
    operator
        .leftTrigger(0.1)
        .whileTrue(
            ClimberCmds.moveClimber(
                operator::getLeftTriggerAxis, SSConstants.Climber.SERVO_POS_OFF, false))
        .onFalse(ClimberCmds.setVoltage(() -> 0));
    operator
        .rightTrigger(0.1)
        .whileTrue(
            ClimberCmds.moveClimber(
                () -> -1 * operator.getRightTriggerAxis(), SSConstants.Climber.SERVO_POS_ON, true))
        .onFalse(ClimberCmds.setVoltage(() -> 0));

    operator.leftBumper().onTrue(SuperStructure.STARTING_CONF.getCommand());
  }

  public void disabledPeriodic() {
    // Safety
    elevator.setTargetHeight(elevator.getCurrentHeight());
    pivot.setTargetAngle(pivot.getCurrentAngle());
    rollers.setRPM(0);
    shoulder.setTargetAngle(shoulder.getCurrentAngle());
    if (visionsubsystem.getPose() != null) {
      if (!hasRanAuto && visionsubsystem.getPose().getTranslation().getX() != 0) {
        driveSubsystem.setPose(
            new Pose2d(
                visionsubsystem.getPose().getTranslation(),
                AllianceFlipUtil.apply(Rotation2d.fromRadians(Math.PI))));
      }
      //   else {
      //     driveSubsystem.setPose(
      //         new Pose2d(visionsubsystem.getPose().getTranslation(),
      // driveSubsystem.getRotation()));
      //   }
    }
  }

  public void setToNormalDrive() {
    DriveCmds.setDefaultDriveCommand(
        driveSubsystem,
        DriveCmds.joystickDrive(
            driveSubsystem,
            () -> -driver.getLeftY(),
            () -> -driver.getLeftX(),
            () -> -driver.getRightX()),
        "Default Joystick Drive");
  }

  public Command setToNormalDriveCmd() {
    return DriveCmds.changeDefaultDriveCommand(
        driveSubsystem,
        DriveCmds.joystickDrive(
            driveSubsystem,
            () -> -driver.getLeftY(),
            () -> -driver.getLeftX(),
            () -> -driver.getRightX()),
        "Default Joystick Drive");
  }

  public Command normalDriveCmd() {
    return DriveCmds.joystickDrive(
        driveSubsystem,
        () -> -driver.getLeftY(),
        () -> -driver.getLeftX(),
        () -> -driver.getRightX());
  }
}
