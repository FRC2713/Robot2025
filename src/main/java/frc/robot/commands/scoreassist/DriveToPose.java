package frc.robot.commands.scoreassist;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.constants.DriveConstants;
import frc.robot.subsystems.constants.ScoreAssistConstants;
import frc.robot.subsystems.drive.Drivetrain;
import java.util.function.Supplier;

public class DriveToPose extends Command {

  private Supplier<Pose2d> targetPose;

  private ProfiledPIDController yscoreAssistController =
      ScoreAssistConstants.assistGains.createTrapezoidalPIDController();
  private ProfiledPIDController xscoreAssistController =
      ScoreAssistConstants.assistGains.createTrapezoidalPIDController();
  private ProfiledPIDController omegascoreAssistController =
      DriveConstants.HeadingControllerConstants.angleGains.createAngularTrapezoidalPIDController();
  private Drivetrain drive;

  private final boolean runInBackground;

  public DriveToPose(Supplier<Pose2d> pose, Drivetrain drive) {
    this(pose, drive, false);
  }

  public DriveToPose(Supplier<Pose2d> pose, Drivetrain drive, boolean runInBackground) {
    this.targetPose = pose;
    this.drive = drive;
    this.runInBackground = runInBackground;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    xscoreAssistController.reset(drive.getPose().getX());
    yscoreAssistController.reset(drive.getPose().getY());
    omegascoreAssistController.reset(drive.getRotation().getRadians());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

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

    // helpful for tuning controllers
    // Logger.recordOutput("ScoreAssist/PIDDriving/TargetPose", targetPose.get());
    // Logger.recordOutput("ScoreAssist/PIDDriving/CommandedSpeeds", speeds);
    // Logger.recordOutput(
    //     "ScoreAssist/PIDDriving/X Current Setpoint",
    //     this.xscoreAssistController.getSetpoint().position);
    // Logger.recordOutput(
    //     "ScoreAssist/PIDDriving/X Current Goal", this.xscoreAssistController.getGoal().position);
    // Logger.recordOutput(
    //     "ScoreAssist/PIDDriving/Y Current Setpoint",
    //     this.xscoreAssistController.getSetpoint().position);
    // Logger.recordOutput(
    //     "ScoreAssist/PIDDriving/Y Current Goal", this.xscoreAssistController.getGoal().position);
    // Logger.recordOutput(
    //     "ScoreAssist/PIDDriving/Theta Current Setpoint",
    //     this.omegascoreAssistController.getSetpoint().position);
    // Logger.recordOutput(
    //     "ScoreAssist/PIDDriving/Theta Current Goal",
    //     this.omegascoreAssistController.getGoal().position);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (this.runInBackground) {
      return false;
    }

    return RobotContainer.scoreAssist.isAtFinalTargetPose();
  }
}
