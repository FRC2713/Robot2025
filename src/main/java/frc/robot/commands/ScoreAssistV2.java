package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.constants.DriveConstants;
import frc.robot.subsystems.drive.Drivetrain;
import frc.robot.util.ScoreNode;
import org.littletonrobotics.junction.Logger;

public class ScoreAssistV2 {
  private static ProfiledPIDController yscoreAssistController =
      DriveConstants.scoreAssistGains.createTrapezoidalPIDController();
  private static ProfiledPIDController xscoreAssistController =
      DriveConstants.scoreAssistGains.createTrapezoidalPIDController();
  private static ProfiledPIDController omegascoreAssistController =
      DriveConstants.HeadingControllerConstants.angleGains.createAngularTrapezoidalPIDController();

  public static Command goScoreAssistV2(ScoreNode node, Drivetrain drive) {
    return Commands.sequence(
        new InstantCommand(
            () -> {
              xscoreAssistController.reset(0);
            }),
        Commands.run(
            () -> {
              var pose = node.getRobotAlignmentPose();
              Logger.recordOutput("ScoreAssistV2/alignToLoc", pose);
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

              double omega = 0;

              //   if (angularError > Units.degreesToRadians(5)) {
              omega =
                  omegascoreAssistController.calculate(
                      drive.getRotation().getRadians(), pose.getRotation().getRadians());
              var angularError = omegascoreAssistController.getPositionError();
              Logger.recordOutput("ScoreAssistV2/angularMeasure", drive.getRotation().getRadians());
              Logger.recordOutput(
                  "ScoreAssistV2/angularGoal", omegascoreAssistController.getGoal().position);
              Logger.recordOutput(
                  "ScoreAssistV2/angularSetpoint",
                  omegascoreAssistController.getSetpoint().position);

              // Logger.recordOutput("ScoreAssistV2/atAngularTarget", false);

              //   } else {
              // Logger.recordOutput("ScoreAssistV2/atAngularTarget", true);
              //   }

              // Convert to field relative speeds & send command
              ChassisSpeeds speeds =
                  new ChassisSpeeds(linearVelocity.getX(), linearVelocity.getY(), omega);

              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      speeds,
                      isFlipped
                          ? drive.getRotation().plus(new Rotation2d(Math.PI))
                          : drive.getRotation()));
              var error = pose.getTranslation().getDistance(drive.getPose().getTranslation());
              Logger.recordOutput("ScoreAssistV2/Error", error);
              Logger.recordOutput("ScoreAssistV2/angular Error", angularError);
            },
            drive));
  }
}
