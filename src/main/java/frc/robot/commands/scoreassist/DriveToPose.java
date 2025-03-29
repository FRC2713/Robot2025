package frc.robot.commands.scoreassist;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.scoreassist.ScoreAssist.ScoreDrivingMode;
import frc.robot.subsystems.constants.DriveConstants;
import frc.robot.subsystems.constants.ScoreAssistConstants;
import frc.robot.subsystems.drive.Drivetrain;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class DriveToPose extends Command {

  private Supplier<Pose2d> targetPose;

  private ProfiledPIDController yscoreAssistController =
      ScoreAssistConstants.assistGains.createTrapezoidalPIDController();
  private ProfiledPIDController xscoreAssistController =
      ScoreAssistConstants.assistGains.createTrapezoidalPIDController();
  private ProfiledPIDController omegascoreAssistController =
      DriveConstants.HeadingControllerConstants.angleGains.createAngularTrapezoidalPIDController();
  private Drivetrain drive;

  private double xError = Double.MAX_VALUE;
  private double yError = Double.MAX_VALUE;
  private double thetaError = Double.MAX_VALUE;

  /** Helper function to calculate the drivetrain's errors to certain targets */
  private void recalculateErrors() {
    Pose2d targetToRobotError =
        targetPose.get().relativeTo(RobotContainer.driveSubsystem.getPose());
    this.xError = targetToRobotError.getX();
    this.yError = targetToRobotError.getY();
    this.thetaError = targetToRobotError.getRotation().getDegrees();

    Logger.recordOutput("ScoreAssist/xErrorInches", Units.metersToInches(this.xError));
    Logger.recordOutput("ScoreAssist/yErrorInches", Units.metersToInches(this.yError));
    Logger.recordOutput("ScoreAssist/thetaError", this.thetaError);
  }

  public DriveToPose(Supplier<Pose2d> pose, Drivetrain drive) {
    this.targetPose = pose;
    this.drive = drive;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (Constants.tuningMode) {
      yscoreAssistController = ScoreAssistConstants.assistGains.createTrapezoidalPIDController();
      xscoreAssistController = ScoreAssistConstants.assistGains.createTrapezoidalPIDController();
      omegascoreAssistController =
          DriveConstants.HeadingControllerConstants.angleGains
              .createAngularTrapezoidalPIDController();
    }
    xscoreAssistController.reset(drive.getPose().getX());
    yscoreAssistController.reset(drive.getPose().getY());
    omegascoreAssistController.reset(drive.getRotation().getRadians());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    recalculateErrors();

    boolean isFlipped =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;
    // Get linear velocity
    Translation2d linearVelocity =
        new Translation2d(
            (isFlipped ? -1 : 1)
                * xscoreAssistController.calculate(
                    drive.getPose().getTranslation().getX(),
                    targetPose.get().getTranslation().getX()),
            (isFlipped ? -1 : 1)
                * yscoreAssistController.calculate(
                    drive.getPose().getTranslation().getY(),
                    targetPose.get().getTranslation().getY()));

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
        "ScoreAssist/Driving/Y Current Setpoint",
        this.yscoreAssistController.getSetpoint().position);
    Logger.recordOutput(
        "ScoreAssist/Driving/Y Current Goal", this.yscoreAssistController.getGoal().position);
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

  /**
   * When the drivetrain is close enough to the target, drive to pose is done
   *
   * @return if drive to pose is done (based on drivetrain position)
   */
  @Override
  public boolean isFinished() {
    boolean slow = RobotContainer.driveSubsystem.getSpeed() < 0.08;
    boolean withinX = Math.abs(this.xError) < ScoreAssistConstants.assistXTolerance.getAsDouble();
    boolean withinY = Math.abs(this.yError) < ScoreAssistConstants.assistYTolerance.getAsDouble();
    boolean withinTheta =
        Math.abs(this.thetaError) < ScoreAssistConstants.assistThetaTolerance.getAsDouble();

    boolean isAtTargetPose =
        RobotContainer.scoreAssist.mode == ScoreDrivingMode.ASSIST
            && withinX
            && withinY
            && withinTheta
            && slow;
    Logger.recordOutput("ScoreAssist/isAtFinalTargetPose", isAtTargetPose);

    return isAtTargetPose;
  }
}
