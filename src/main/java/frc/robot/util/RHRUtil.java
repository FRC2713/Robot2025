package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.RobotContainer;

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
}
