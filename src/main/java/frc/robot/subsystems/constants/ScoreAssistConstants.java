package frc.robot.subsystems.constants;

import frc.robot.util.ControlGains;
import frc.robot.util.LoggedTunableGains;
import frc.robot.util.LoggedTunableNumber;

public class ScoreAssistConstants {

  public static final LoggedTunableGains scoreAssistGains =
      new LoggedTunableGains("ScoreAssist", new ControlGains().p(2).d(0).trapezoidal(0.1, 0, 0));

  public static final LoggedTunableNumber scoreAsssistTolerance =
      new LoggedTunableNumber("ScoreAssist/Tolerance", 0.03);

  public static final LoggedTunableNumber scoreAsssistActivationThreshold =
      new LoggedTunableNumber("ScoreAssist/Activation Threshold", 1.25);
}
