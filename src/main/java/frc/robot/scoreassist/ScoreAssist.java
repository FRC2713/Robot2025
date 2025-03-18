package frc.robot.scoreassist;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.constants.DriveConstants;
import frc.robot.subsystems.drive.Drivetrain;
import frc.robot.util.ScoreNode;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ScoreAssist extends Command {
  @Getter private boolean readyForPrep = false;
  @Getter private boolean readyForScore = false;
  @Getter private double error = Double.MAX_VALUE;
  private ScoreNode node;

  private ProfiledPIDController yscoreAssistController =
      DriveConstants.scoreAssistGains.createTrapezoidalPIDController();
  private ProfiledPIDController xscoreAssistController =
      DriveConstants.scoreAssistGains.createTrapezoidalPIDController();
  private ProfiledPIDController omegascoreAssistController =
      DriveConstants.HeadingControllerConstants.angleGains.createAngularTrapezoidalPIDController();
  private Drivetrain drive;

  /** Creates a new ScoreAssist. */
  public ScoreAssist(ScoreNode node, Drivetrain drive) {
    this.node = node;
    this.drive = drive;
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
    var pose = node.getRobotAlignmentPose();
    readyForPrep = true;
    Logger.recordOutput("ScoreAssist/alignToLoc", pose);

    boolean isFlipped =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;
    // Get linear velocity
    Translation2d linearVelocity =
        new Translation2d(
            (isFlipped ? -1 : 1)
                * xscoreAssistController.calculate(
                    drive.getPose().getTranslation().getX(), pose.getTranslation().getX()),
            (isFlipped ? -1 : 1)
                * yscoreAssistController.calculate(
                    drive.getPose().getTranslation().getY(), pose.getTranslation().getY()));

    // Calculate angular speed
    double omega =
        omegascoreAssistController.calculate(
            drive.getRotation().getRadians(), pose.getRotation().getRadians());

    // Convert to field relative speeds & send command
    ChassisSpeeds speeds = new ChassisSpeeds(linearVelocity.getX(), linearVelocity.getY(), omega);

    Logger.recordOutput("ScoreAssist/CommandedSpeeds", speeds);

    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            speeds,
            isFlipped ? drive.getRotation().plus(new Rotation2d(Math.PI)) : drive.getRotation()));

    error = pose.getTranslation().getDistance(drive.getPose().getTranslation());
    Logger.recordOutput("ScoreAssist/Error", error);

    if (error < Units.inchesToMeters(1)) {
      readyForScore = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
