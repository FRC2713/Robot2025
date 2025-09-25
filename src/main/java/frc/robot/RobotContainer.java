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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.commands.DriveCmds;
// See line 232 for why this is commented out -Owen
// import frc.robot.commands.autos.CenterAutoBarge;
import frc.robot.commands.autos.CenterAutoOnePiece;
import frc.robot.commands.autos.DriveTesting;
import frc.robot.commands.autos.ScoreLotsOfCoral;
import frc.robot.commands.autos.ScoreLotsOfCoralFlipped;
import frc.robot.generated.TunerConstants;
import frc.robot.oi.DeveloperControls; // Added import for DeveloperControls
import frc.robot.oi.DriverControls;
import frc.robot.oi.OperatorControls;
import frc.robot.scoreassist.ClimbAssist;
import frc.robot.scoreassist.ScoreAssist;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOSim;
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
import frc.robot.subsystems.endEffector.EndEffector;
import frc.robot.subsystems.endEffector.EndEffectorIO;
import frc.robot.subsystems.endEffector.EndEffectorIOSim;
import frc.robot.subsystems.endEffector.EndEffectorIOSparks;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.pivot.PivotIO;
import frc.robot.subsystems.pivot.PivotIOKrakens;
import frc.robot.subsystems.pivot.PivotIOSim;
import frc.robot.subsystems.shoulder.Shoulder;
import frc.robot.subsystems.shoulder.ShoulderIO;
import frc.robot.subsystems.shoulder.ShoulderIOKrakens;
import frc.robot.subsystems.shoulder.ShoulderIOSim;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOLimelights;
import frc.robot.subsystems.vision.VisionIOOdometry;
import frc.robot.subsystems.vision.VisionIOPoseEstimator;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.RHRHolonomicDriveController;
import java.util.Arrays;
import org.littletonrobotics.junction.Logger;

public class RobotContainer {
  // Subsystems
  public static Drivetrain driveSubsystem;
  public static Elevator elevator;
  public static Shoulder shoulder;
  public static Pivot pivot;
  public static Climber climber;
  public static EndEffector endEffector;

  // Xbox Controllers
  public static DriverControls driverControls = new DriverControls();
  public static OperatorControls operatorControls = new OperatorControls();
  public static DeveloperControls devControls = new DeveloperControls();

  private static boolean hasRanAuto = false;

  // Dashboard inputs
  public final AutoChooser autoChooser;

  // For Choreo
  private final AutoFactory choreoAutoFactory;
  public static final RHRHolonomicDriveController otfController =
      new RHRHolonomicDriveController(
          OTFConstants.translationPID, // Translation PID constants
          OTFConstants.rotationPID, // Rotation PID constants
          OTFConstants.translationTolerance // Translation Tolerenace
          );

  // For teleop automation
  public static ScoreAssist scoreAssist;
  public static ClimbAssist climbAssist;
  public static Vision visionsubsystem;
  public static boolean disableReefAlign = false;
  public static boolean disableSourceAlign = true;
  public static boolean autoScorePathing = false;

  public static LoggedTunableNumber autoWait = new LoggedTunableNumber("auto wait", 0);

  // private Trigger climbPrepTrigger = new
  // Trigger(ScoreAssistOld.getInstance()::shouldClimbPrep);

  public RobotContainer() { // Start subsystems
    switch (Constants.currentMode) {
      case REAL:
        driveSubsystem =
            new Drivetrain(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));

        elevator = new Elevator();
        pivot = new Pivot(new PivotIOKrakens());
        shoulder = new Shoulder(new ShoulderIOKrakens());
        climber = new Climber(new ClimberIOSparks());
        endEffector = new EndEffector(new EndEffectorIOSparks());
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
        elevator = new Elevator();
        shoulder = new Shoulder(new ShoulderIOSim());
        climber = new Climber(new ClimberIOSim());
        endEffector = new EndEffector(new EndEffectorIOSim());
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
        elevator = new Elevator();
        pivot = new Pivot(new PivotIO() {});
        shoulder = new Shoulder(new ShoulderIO() {});
        climber = new Climber(new ClimberIO() {});
        endEffector = new EndEffector(new EndEffectorIO() {});
        break;
    }
    visionsubsystem =
        new Vision(
            // new VisionIO() {}
            VisionConstants.ACTIVE_VISION_OPTION == VisionOptions.SLAMDUNK_WHEEL_ODOMETRY
                ? new VisionIOOdometry()
                : ((VisionConstants.ACTIVE_VISION_OPTION == VisionOptions.SLAMDUNK
                        || VisionConstants.ACTIVE_VISION_OPTION
                            == VisionOptions.SLAMDUNK_MEGATAG2_MERGED)
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
        "Centre - One Piece",
        () -> CenterAutoOnePiece.getRoutine(choreoAutoFactory, driveSubsystem));
    // Note: commented out since it was causing errors that I didn't have time to look into -Owen
    /*autoChooser.addRoutine(
    "Centre - Barge", () -> CenterAutoBarge.getRoutine(choreoAutoFactory, driveSubsystem));*/
    autoChooser.addRoutine(
        "Right - Score Lots Of Coral",
        () -> ScoreLotsOfCoral.getRoutine(choreoAutoFactory, driveSubsystem));
    autoChooser.addRoutine(
        "Left - Score Lots Of Coral",
        () ->
            ScoreLotsOfCoralFlipped.getRoutine(choreoAutoFactory, driveSubsystem)); // tODO: rename
    autoChooser.addRoutine(
        "Drive Testing", () -> DriveTesting.getRoutine(choreoAutoFactory, driveSubsystem));

    // Uncomment for swerve drive characterization
    // autoChooser.addCmd(
    //     "Drive Simple FF Characterization",
    //     () -> DriveCmds.feedforwardCharacterization(driveSubsystem));
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
    climbAssist = new ClimbAssist();

    // Configure the button bindings
    driverControls.configureButtonBindings();
    driverControls.configureTriggers();
    operatorControls.configureButtonBindings();
    operatorControls.configureTriggers();
    devControls.configureButtonBindings();
    devControls.configureTriggers();

    // Default command, normal field-relative drive
    driverControls.setToNormalDrive();

    Logger.recordOutput("FieldConstants/centerFaces", FieldConstants.Reef.centerFaces);
    Logger.recordOutput("FieldConstants/processorFace", FieldConstants.Processor.centerFace);
  }

  public void disabledPeriodic() {
    // Safety
    elevator.setTargetHeight(elevator.getCurrentHeight());
    pivot.setTargetAngle(pivot.getCurrentAngle());
    endEffector.setCoralRPM(0);
    endEffector.setAlgaeRPM(0);
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
}
