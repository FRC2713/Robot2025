package frc.robot.subsystems.constants;

import edu.wpi.first.math.util.Units;
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
      new LoggedTunableGains("ScoreAssist", new ControlGains().p(10).d(0).trapezoidal(2.5, 5, 0));

  public static final LoggedTunableNumber assistXTolerance =
      new LoggedTunableNumber("ScoreAssist/X Tolerance", Units.inchesToMeters(0.75));

  public static final LoggedTunableNumber assistYTolerance =
      new LoggedTunableNumber("ScoreAssist/Y Tolerance", Units.inchesToMeters(2.5));

  public static final LoggedTunableNumber assistThetaTolerance =
      new LoggedTunableNumber("ScoreAssist/Theta Tolerance", 2); // degrees
}
