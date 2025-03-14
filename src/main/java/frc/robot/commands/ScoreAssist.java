package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.networktables.StringTopic;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.SSConstants;
import frc.robot.subsystems.constants.DriveConstants;
import frc.robot.subsystems.constants.DriveConstants.HeadingControllerConstants;
import frc.robot.subsystems.drive.Drivetrain;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.ScoreLoc;
import frc.robot.util.ScoreNode;
import java.util.Optional;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;

public class ScoreAssist {
  @Getter @Setter private Pose2d closestLocPose = null;
  public boolean hasStartedCommand = false;
  private static ScoreAssist INSTANCE = null;
  public double error = -1;
  private NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private StringTopic topic = inst.getStringTopic("/scoreassist/goto");
  private StringSubscriber sub;
  public Optional<ScoreLoc> reefTrackerLoc = Optional.empty();

  private ScoreAssist() {
    sub = topic.subscribe("none");
  }

  public static ScoreAssist getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new ScoreAssist();
    }
    return INSTANCE;
  }

  private ProfiledPIDController yscoreAssistController =
      DriveConstants.scoreAssistGains.createTrapezoidalPIDController();
  private ProfiledPIDController xscoreAssistController =
      DriveConstants.scoreAssistGains.createTrapezoidalPIDController();
  private ProfiledPIDController omegascoreAssistController =
      DriveConstants.HeadingControllerConstants.angleGains.createAngularTrapezoidalPIDController();

  public Pose2d getClosest() {
    var pose = RobotContainer.driveSubsystem.getPose();
    ScoreNode closestNode = null;
    double closestDistance = Double.MAX_VALUE;

    for (ScoreNode node : ScoreNode.values()) {
      double distance =
          pose.getTranslation()
              .getDistance(AllianceFlipUtil.apply(node.getRobotAlignmentPose().getTranslation()));
      if (distance < closestDistance) {
        closestDistance = distance;
        closestNode = node;
      }
    }

    return closestNode != null ? AllianceFlipUtil.apply(closestNode.getRobotAlignmentPose()) : null;
  }

  private boolean isFinished = false;

  public boolean hasFinished() {
    return error < Units.inchesToMeters(1);
  }

  public Command alignTo(Drivetrain drive) {
    return Commands.parallel(
            Commands.runOnce(
                () -> {
                  if (reefTrackerLoc.isPresent()) {
                    reefTrackerLoc.get().getLevel().getPrepCommand().get().schedule();
                  }
                }),
            Commands.run(
                () -> {
                  RobotContainer.disableReefAlign = true;
                  Pose2d pose = null;
                  if (reefTrackerLoc.isEmpty()) {
                    if (getClosestLocPose() == null) {
                      pose = getClosest();
                      setClosestLocPose(pose);
                    } else {
                      pose = getClosestLocPose();
                    }
                  } else {
                    pose = reefTrackerLoc.get().getNode().getRobotAlignmentPose();
                  }
                  Logger.recordOutput("ScoreAssist/alignToLoc", pose);
                  boolean isFlipped =
                      DriverStation.getAlliance().isPresent()
                          && DriverStation.getAlliance().get() == Alliance.Red;
                  // Get linear velocity
                  Translation2d linearVelocity =
                      new Translation2d(
                          (isFlipped ? -1 : 1)
                              * xscoreAssistController.calculate(
                                  drive.getPose().getTranslation().getX(),
                                  pose.getTranslation().getX()),
                          (isFlipped ? -1 : 1)
                              * yscoreAssistController.calculate(
                                  drive.getPose().getTranslation().getY(),
                                  pose.getTranslation().getY()));

                  Logger.recordOutput("ScoreAssist/CommandedLinearVelocity", linearVelocity);

                  // Calculate angular speed
                  double omega =
                      omegascoreAssistController.calculate(
                          drive.getRotation().getRadians(), pose.getRotation().getRadians());

                  // Convert to field relative speeds & send command
                  ChassisSpeeds speeds =
                      new ChassisSpeeds(linearVelocity.getX(), linearVelocity.getY(), omega);

                  drive.runVelocity(
                      ChassisSpeeds.fromFieldRelativeSpeeds(
                          speeds,
                          isFlipped
                              ? drive.getRotation().plus(new Rotation2d(Math.PI))
                              : drive.getRotation()));
                  error = pose.getTranslation().getDistance(drive.getPose().getTranslation());
                  Logger.recordOutput("ScoreAssist/Error", error);
                  if (error < Units.inchesToMeters(1)) {
                    isFinished = true;
                    if (!hasStartedCommand) {
                      hasStartedCommand = true;
                      if (reefTrackerLoc.isPresent()) {
                        reefTrackerLoc
                            .get()
                            .getLevel()
                            .getScoreCommand()
                            .get()
                            .andThen(
                                Commands.sequence(
                                    Commands.waitSeconds(
                                        SSConstants.Auto.L4_SCORE_DELAY.getAsDouble()),
                                    SuperStructure.CORAL_SCORE.getCommand(),
                                    Commands.waitSeconds(
                                        SSConstants.Auto.L4_POST_SCORE_DELAY.getAsDouble()),
                                    SuperStructure.SOURCE_CORAL_INTAKE.getCommand()))
                            .schedule();
                      }
                    }
                  }
                  Logger.recordOutput("ScoreAssist/isFinished", isFinished);
                },
                drive))

        // Reset PID controller when command starts
        .beforeStarting(
            () -> {
              xscoreAssistController.reset(drive.getPose().getX());
              yscoreAssistController.reset(drive.getPose().getY());
              HeadingControllerConstants.angleController.reset(drive.getRotation().getRadians());
            })
        .finallyDo(
            () -> {
              setClosestLocPose(null);
              error = -1;
              hasStartedCommand = false;
              isFinished = false;
            });
  }

  public Command goClosest(Drivetrain drive) {
    return alignTo(drive);
  }

  public Command goReefTracker(Drivetrain drive) {
    return alignTo(drive);
  }

  public void periodic() {
    if (DriveConstants.scoreAssistGains.hasChanged(hashCode())) {
      yscoreAssistController = DriveConstants.scoreAssistGains.createTrapezoidalPIDController();
      xscoreAssistController = DriveConstants.scoreAssistGains.createTrapezoidalPIDController();
    }
    if (DriveConstants.HeadingControllerConstants.angleGains.hasChanged(hashCode())) {
      omegascoreAssistController =
          DriveConstants.HeadingControllerConstants.angleGains
              .createAngularTrapezoidalPIDController();
      omegascoreAssistController.enableContinuousInput(-Math.PI, Math.PI);
    }
    var loc = ScoreLoc.parseFromNT(sub.get("none"));
    if (loc != null) {

      Logger.recordOutput("ScoreAssist/ReefTrackerLoc", loc.toString());
      Logger.recordOutput("ScoreAssist/ReefTrackerPose", loc.getNode().getRobotAlignmentPose());
      reefTrackerLoc = Optional.of(loc);
    } else {
      reefTrackerLoc = Optional.empty();
      Logger.recordOutput("ScoreAssist/ReefTrackerLoc", "null");
    }
  }
}
