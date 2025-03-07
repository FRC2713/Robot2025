package frc.robot.util;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class RHRHolonomicDriveController extends PPHolonomicDriveController {

  private final PIDController customXController;
  private final PIDController customYController;

  public RHRHolonomicDriveController(
      PIDConstants translationConstants,
      PIDConstants rotationConstants,
      double translationTolerance,
      double period) {
    super(translationConstants, rotationConstants, period);

    this.customXController =
        new PIDController(
            translationConstants.kP, translationConstants.kI, translationConstants.kD, period);
    this.customXController.setIntegratorRange(
        -translationConstants.iZone, translationConstants.iZone);
    this.customXController.setTolerance(translationTolerance);

    this.customYController =
        new PIDController(
            translationConstants.kP, translationConstants.kI, translationConstants.kD, period);
    this.customYController.setIntegratorRange(
        -translationConstants.iZone, translationConstants.iZone);
    this.customYController.setTolerance(translationTolerance);
  }

  public RHRHolonomicDriveController(
      PIDConstants translationConstants, PIDConstants rotationConstants) {
    this(translationConstants, rotationConstants, 0.05, 0.02);
  }

  public RHRHolonomicDriveController(
      PIDConstants translationConstants,
      PIDConstants rotationConstants,
      double translationTolerance) {
    this(translationConstants, rotationConstants, translationTolerance, 0.02);
  }

  @Override
  public ChassisSpeeds calculateRobotRelativeSpeeds(
      Pose2d currentPose, PathPlannerTrajectoryState targetState) {

    double xFeedback =
        this.customXController.calculate(currentPose.getX(), targetState.pose.getX());
    double yFeedback =
        this.customYController.calculate(currentPose.getY(), targetState.pose.getY());

    PPHolonomicDriveController.overrideXFeedback(() -> xFeedback);
    PPHolonomicDriveController.overrideYFeedback(() -> yFeedback);

    return super.calculateRobotRelativeSpeeds(currentPose, targetState);
  }

  @Override
  public void reset(Pose2d currentPose, ChassisSpeeds currentSpeeds) {
    super.reset(currentPose, currentSpeeds);
    this.customXController.reset();
    this.customYController.reset();
  }
}
