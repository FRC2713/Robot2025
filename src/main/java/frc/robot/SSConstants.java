package frc.robot;

import frc.robot.util.LoggedTunableNumber;

public class SSConstants {
  public class Elevator {
    public static final LoggedTunableNumber L1_CORAL_SCORE_HEIGHT_IN =
        new LoggedTunableNumber("Elevator/SS/L1 Score Height", 28);
    public static final LoggedTunableNumber L1_CORAL_PREP_HEIGHT_IN =
        new LoggedTunableNumber("Elevator/SS/L1 Prep Height", 28);

    public static final LoggedTunableNumber L2_CORAL_SCORE_HEIGHT_IN =
        new LoggedTunableNumber("Elevator/SS/L2 Score Height", 50);

    public static final LoggedTunableNumber L2_CORAL_PREP_HEIGHT_IN =
        new LoggedTunableNumber("Elevator/SS/L2 Prep Height", 50);
  }

  public class Pivot {
    public static final LoggedTunableNumber L1_CORAL_SCORE_ANGLE_DEG =
        new LoggedTunableNumber("Pivot/SS/L1 Score Angle", 35);
    public static final LoggedTunableNumber L1_CORAL_PREP_ANGLE_DEG =
        new LoggedTunableNumber("Pivot/SS/L1 Prep Angle", 35);

    public static final LoggedTunableNumber L2_CORAL_SCORE_ANGLE_DEG =
        new LoggedTunableNumber("Pivot/SS/L2 Score Angle", 35);

    public static final LoggedTunableNumber L2_CORAL_PREP_ANGLE_DEG =
        new LoggedTunableNumber("Pivot/SS/L2 Prep Angle", 35);
  }

  public class Roller {
    public static final LoggedTunableNumber L1_CORAL_SCORE_SPEED =
        new LoggedTunableNumber("Roller/SS/L1 Score Speed", 1000);

    public static final LoggedTunableNumber L2_CORAL_SCORE_SPEED =
        new LoggedTunableNumber("Roller/SS/L2 Score Speed", 1000);
  }
}
