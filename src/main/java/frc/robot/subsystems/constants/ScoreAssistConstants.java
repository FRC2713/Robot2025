package frc.robot.subsystems.constants;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.util.Units;
import frc.robot.RobotContainer;
import frc.robot.util.ControlGains;
import frc.robot.util.LoggedTunableGains;
import frc.robot.util.LoggedTunableNumber;

public class ScoreAssistConstants {

  public static final PathConstraints pathFindindConstraints =
      new PathConstraints(
          2.75,
          5.0,
          RobotContainer.driveSubsystem.getMaxAngularSpeedRadPerSec(),
          Units.degreesToRadians(720));

  public static final LoggedTunableGains assistGains =
      new LoggedTunableGains("ScoreAssist", new ControlGains().p(10).d(0).trapezoidal(2.5, 5, 0));

  public static final LoggedTunableNumber assistXTolerance =
      new LoggedTunableNumber("ScoreAssist/X Tolerance", Units.inchesToMeters(1));

  public static final LoggedTunableNumber assistYTolerance =
      new LoggedTunableNumber("ScoreAssist/Y Tolerance", Units.inchesToMeters(2.5));

  public static final LoggedTunableNumber assistThetaTolerance =
      new LoggedTunableNumber("ScoreAssist/Theta Tolerance", 2); // degrees

  public static final LoggedTunableNumber pathDistTolerance =
      new LoggedTunableNumber("ScoreAssist/Activation Threshold", 1);
}
