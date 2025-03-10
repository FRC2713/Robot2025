package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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
  private static ScoreAssist INSTANCE = null;

  public static ScoreAssist getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new ScoreAssist();
    }
    return INSTANCE;
  }

  public Pose2d getClosest() {
    var pose = RobotContainer.driveSubsystem.getPose();
    ScoreNode closestNode = null;
    double closestDistance = Double.MAX_VALUE;

    for (ScoreNode node : ScoreNode.values()) {
      double distance = pose.getTranslation().getDistance(node.getRobotAlignmentPose().getTranslation());
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
              Logger.recordOutput("ScoreAssit/closestsLoc", closest);
              // Get linear velocity
              Translation2d linearVelocity =
                  new Translation2d(
                      DriveConstants.scoreAssistController.calculate(
                          drive.getPose().getTranslation().getX(), closest.getTranslation().getX()),
                      DriveConstants.scoreAssistController.calculate(
                          drive.getPose().getTranslation().getY(),
                          closest.getTranslation().getY()));

            Logger.recordOutput("ScoreAssist/CommandedLinearVelocity", linearVelocity);

              // Calculate angular speed
              double omega =
                  HeadingControllerConstants.angleController.calculate(
                      drive.getRotation().getRadians(), closest.getRotation().getRadians());

              // Convert to field relative speeds & send command
              ChassisSpeeds speeds =
                  new ChassisSpeeds(
                      linearVelocity.getX(),
                      linearVelocity.getY(),
                      omega);
              boolean isFlipped =
                  DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == Alliance.Red;
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      speeds,
                      isFlipped
                          ? drive.getRotation().plus(new Rotation2d(Math.PI))
                          : drive.getRotation()));
            },
            drive)

        // Reset PID controller when command starts
        .beforeStarting(
            () ->
                HeadingControllerConstants.angleController.reset(drive.getRotation().getRadians()))
        .finallyDo(() -> setClosestLocPose(null));
  }

  public void periodic() {}
}
