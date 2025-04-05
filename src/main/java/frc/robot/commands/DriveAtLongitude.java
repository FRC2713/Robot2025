package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.constants.DriveConstants;
import frc.robot.subsystems.constants.ScoreAssistConstants;
import frc.robot.subsystems.drive.Drivetrain;
import frc.robot.util.AllianceFlipUtil;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class DriveAtLongitude extends Command {
  private Supplier<Pose2d> targetPose;

  private ProfiledPIDController xscoreAssistController =
      ScoreAssistConstants.assistGains.createTrapezoidalPIDController();
  private ProfiledPIDController omegascoreAssistController =
      DriveConstants.HeadingControllerConstants.angleGains.createAngularTrapezoidalPIDController();
  private Drivetrain drive;

  // TODO: Use library
  public static double normalizeAngle(double angle) {
    angle %= 360;
    if (angle < 0) {
      angle += 360;
    }
    return angle;
  }

  public static boolean doBackwards(Pose2d pose) {
    var targetRot = normalizeAngle(pose.getRotation().getDegrees());
    var robotRot =
        normalizeAngle(
            RobotContainer.driveSubsystem.getRotation().getDegrees());

    var less =
        (getAngularDistance(normalizeAngle(targetRot + 180), robotRot)
            < getAngularDistance(targetRot, robotRot));
    var targetX = pose.getTranslation().getX();
    var currentX = RobotContainer.driveSubsystem.getTranslation().getX();

    var invert =
        Math.abs(currentX - targetX) > Math.abs(currentX - (FieldConstants.fieldLength - targetX));
    Logger.recordOutput("DriveAtLongitude/invert", invert);
    Logger.recordOutput("DriveAtLongitude/less", less);

    return invert ? !less : less;
  }

  public static double getAngularDistance(double a, double b) {
    double diff = Math.abs(a - b) % 360;
    return Math.min(diff, 360 - diff);
  }

  public DriveAtLongitude(Supplier<Pose2d> pose, Drivetrain drive) {
    this.targetPose =
        () -> {
          if (doBackwards(pose.get())) {
            return new Pose2d(
                pose.get().getTranslation(),
                pose.get().getRotation().plus(Rotation2d.fromDegrees(180)));
          } else return pose.get();
        };
    this.drive = drive;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    xscoreAssistController.reset(drive.getPose().getX());
    omegascoreAssistController.reset(drive.getRotation().getRadians());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    boolean isFlipped =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;

    var driverControls =
        DriveCmds.getLinearVelocityFromJoysticks(
            -RobotContainer.driverControls.getLeftY(), -RobotContainer.driverControls.getLeftX());
    // Get linear velocity
    Translation2d linearVelocity =
        new Translation2d(
            (isFlipped ? -1 : 1)
                * xscoreAssistController.calculate(
                    drive.getPose().getTranslation().getX(),
                    targetPose.get().getTranslation().getX()),
            driverControls.getY() * (drive.getMaxLinearSpeedMetersPerSec() * 0.8));

    // Calculate angular speed
    double omega =
        omegascoreAssistController.calculate(
            drive.getRotation().getRadians(), targetPose.get().getRotation().getRadians());

    // Convert to field relative speeds & send command
    ChassisSpeeds speeds = new ChassisSpeeds(linearVelocity.getX(), linearVelocity.getY(), omega);

    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            speeds,
            isFlipped ? drive.getRotation().plus(new Rotation2d(Math.PI)) : drive.getRotation()));

    Logger.recordOutput("ScoreAssist/Driving/TargetPose", targetPose.get());
    Logger.recordOutput("ScoreAssist/Driving/CommandedSpeeds", speeds);
    Logger.recordOutput(
        "ScoreAssist/Driving/X Current Setpoint",
        this.xscoreAssistController.getSetpoint().position);
    Logger.recordOutput(
        "ScoreAssist/Driving/X Current Goal", this.xscoreAssistController.getGoal().position);
    Logger.recordOutput(
        "ScoreAssist/Driving/Theta Current Setpoint",
        this.omegascoreAssistController.getSetpoint().position);
    Logger.recordOutput(
        "ScoreAssist/Driving/Theta Current Goal",
        this.omegascoreAssistController.getGoal().position);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
