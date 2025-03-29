package frc.robot.subsystems.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.FieldConstants;
import frc.robot.util.ControlGains;
import frc.robot.util.LoggedTunableGains;
import frc.robot.util.LoggedTunableNumber;

public class ScoreAssistConstants {

  // For Path finding
  public static final LoggedTunableNumber pathConstraintVelocityMPS =
      new LoggedTunableNumber("ScoreAssist/Pathfind/maxVelocityMPS", 2.5);

  public static final LoggedTunableNumber pathConstraintaccel =
      new LoggedTunableNumber("ScoreAssist/Pathfind/constraintAccel", 3.0);

  public static final LoggedTunableNumber pathDistTolerance =
      new LoggedTunableNumber("ScoreAssist/Activation Threshold", 1);

  // For ProfiledPID assistance
  public static final LoggedTunableGains assistGains =
      new LoggedTunableGains("ScoreAssist", new ControlGains().p(8).d(1.2).trapezoidal(2., 6, 0));

  public static final LoggedTunableNumber assistXTolerance =
      new LoggedTunableNumber("ScoreAssist/X Tolerance", Units.inchesToMeters(0.75));

  public static final LoggedTunableNumber assistYTolerance =
      new LoggedTunableNumber("ScoreAssist/Y Tolerance", Units.inchesToMeters(2.5));

  public static final LoggedTunableNumber assistThetaTolerance =
      new LoggedTunableNumber("ScoreAssist/Theta Tolerance", 3); // degrees

  public static final Pose2d processorPose =
      FieldConstants.Processor.centerFace.transformBy(new Transform2d(1, 0, new Rotation2d()));

  public static final Pose2d bargeAlignmentX =
      new Pose2d(new Translation2d(7.743555545806885, -1), new Rotation2d(Math.PI));
}
