package frc.robot.util;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.FieldConstants;
import frc.robot.RobotContainer;
import java.util.ArrayList;

public class RHRUtil {
  public static double speed(Twist2d twist) {
    return Math.sqrt(Math.pow(twist.dx, 2) + Math.pow(twist.dy, 2));
  }

  public static double modeDependentDouble(double real) {
    return real;
  }

  public static double modeDependentDouble(double real, double sim) {
    return Constants.currentMode == Mode.SIM ? sim : real;
  }

  public static Command resetRotationIfReal(Pose2d initialPose) {
    return Constants.currentMode == Mode.SIM
        ? new InstantCommand(() -> RobotContainer.driveSubsystem.resetOdometry(initialPose))
        : new InstantCommand(
            () ->
                RobotContainer.driveSubsystem.resetOdometry(
                    new Pose2d(
                        RobotContainer.driveSubsystem.getPose().getTranslation(),
                        initialPose.getRotation())));
  }

  public static Pose2d integrate(ChassisSpeeds speeds, Pose2d initialPose, double delta_t) {
    return initialPose.exp(speeds.toTwist2d(delta_t));
  }

  /**
   * Returns this trajectory, mirrored across the HORIZONTAL field midline.
   *
   * @param trajectory the trajectory to mirror.
   * @return trajectory, mirrored across the HORIZONTAL field midline.
   */
  public static AutoTrajectory flipHorizontal(AutoTrajectory autotrajectory, AutoRoutine routine) {
    Trajectory<SwerveSample> trajectory = autotrajectory.getRawTrajectory();
    var flippedStates = new ArrayList<SwerveSample>();
    for (var state : trajectory.samples()) {
      flippedStates.add(
          new SwerveSample(
              state.t,
              state.x,
              FieldConstants.fieldWidth - (state.y),
              -state.heading,
              state.vx,
              -state.vy,
              -state.omega,
              state.ax,
              -state.ay,
              -state.alpha,
              // TODO: VERIFY THIS
              // FL, FR, BL, BR
              // Mirrored
              // FR, FL, BR, BL
              new double[] {
                state.moduleForcesX()[1],
                state.moduleForcesX()[0],
                state.moduleForcesX()[3],
                state.moduleForcesX()[2]
              },
              // FL, FR, BL, BR
              // Mirrored
              // -FR, -FL, -BR, -BL
              new double[] {
                -state.moduleForcesY()[1],
                -state.moduleForcesY()[0],
                -state.moduleForcesY()[3],
                -state.moduleForcesY()[2]
              }));
    }
    var newtraj =
        new Trajectory<SwerveSample>(
            trajectory.name(), flippedStates, trajectory.splits(), trajectory.events());
    return routine.trajectory(newtraj);
  }
}
