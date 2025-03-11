package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.subsystems.constants.DriveConstants;
import frc.robot.subsystems.constants.DriveConstants.HeadingControllerConstants;
import frc.robot.subsystems.drive.Drivetrain;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.ScoreNode;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;

public class ScoreAssist {
  @Getter @Setter private Pose2d closestLocPose = null;
  public boolean hasStartedCommand = false;
  private static ScoreAssist INSTANCE = null;

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

  public Command goClosest(Drivetrain drive) {
    return Commands.run(
            () -> {
              Pose2d closest = null;
              if (getClosestLocPose() == null) {
                closest = getClosest();
                setClosestLocPose(closest);
              } else {
                closest = getClosestLocPose();
              }
              Logger.recordOutput("ScoreAssist/closestLoc", closest);
              boolean isFlipped =
                  DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == Alliance.Red;
              // Get linear velocity
              Translation2d linearVelocity =
                  new Translation2d(
                      (isFlipped ? -1 : 1)
                          * xscoreAssistController.calculate(
                              drive.getPose().getTranslation().getX(),
                              closest.getTranslation().getX()),
                      (isFlipped ? -1 : 1)
                          * yscoreAssistController.calculate(
                              drive.getPose().getTranslation().getY(),
                              closest.getTranslation().getY()));

              Logger.recordOutput("ScoreAssist/CommandedLinearVelocity", linearVelocity);

              // Calculate angular speed
              double omega =
                  omegascoreAssistController.calculate(
                      drive.getRotation().getRadians(), closest.getRotation().getRadians());

              // Convert to field relative speeds & send command
              ChassisSpeeds speeds =
                  new ChassisSpeeds(linearVelocity.getX(), linearVelocity.getY(), omega);

              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      speeds,
                      isFlipped
                          ? drive.getRotation().plus(new Rotation2d(Math.PI))
                          : drive.getRotation()));
              double error = closest.getTranslation().getDistance(drive.getPose().getTranslation());
              Logger.recordOutput("ScoreAssist/Error", error);
              if (error < Units.inchesToMeters(1) && !hasStartedCommand) {
                SuperStructure.L4_PREP
                    .getCommand()
                    .andThen(SuperStructure.CORAL_SCORE.getCommand())
                    .schedule();
                hasStartedCommand = true;
              }
            },
            drive)

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
              hasStartedCommand = false;
            });
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
  }
}
