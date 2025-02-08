package frc.robot;

import frc.robot.util.LoggedTunableNumber;

public class SSConstants {
  public class Elevator {
    public static final LoggedTunableNumber SOURCE_CORAL_INTAKE_HEIGHT_IN =
        new LoggedTunableNumber("Elevator/SS/Source Coral Intake Height", 20);

    public static final LoggedTunableNumber L1_ALGAE_GRAB_HEIGHT_IN =
        new LoggedTunableNumber("Elevator/SS/L1 Algae Grab Height", 0);

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
    public static final LoggedTunableNumber SOURCE_CORAL_INTAKE_ANGLE_DEG =
        new LoggedTunableNumber("Pivot/SS/Source Intake Angle", 35);

    public static final LoggedTunableNumber L1_ALGAE_GRAB_DEG =
        new LoggedTunableNumber("Pivot/SS/L1 Algae Grab Angle", 35);

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

    public static final LoggedTunableNumber SOURCE_CORAL_INTAKE_SPEED =
        new LoggedTunableNumber("Roller/SS/Source Intake Speed", 1000);

    public static final LoggedTunableNumber L1_ALGAE_GRAB_SPEED =
        new LoggedTunableNumber("Roller/SS/Algae Grab Speed", -1000);
  }
}
