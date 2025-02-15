package frc.robot;

import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;

public class SSConstants {
  public class Elevator {
    public static final LoggedTunableNumber SOURCE_CORAL_INTAKE_HEIGHT_IN =
        new LoggedTunableNumber("Elevator/SS/Source Coral Intake Height", 0);

    public static final LoggedTunableNumber L1_ALGAE_GRAB_HEIGHT_IN =
        new LoggedTunableNumber("Elevator/SS/L1 Algae Grab Height", 22);

    public static final LoggedTunableNumber L3_ALGAE_GRAB_HEIGHT_IN =
        new LoggedTunableNumber("Elevator/SS/L3 Algae Grab Height", 35);

    public static final LoggedTunableNumber L1_CORAL_SCORE_HEIGHT_IN =
        new LoggedTunableNumber("Elevator/SS/L1 Score Height", 5);
    public static final LoggedTunableNumber L1_CORAL_PREP_HEIGHT_IN =
        new LoggedTunableNumber("Elevator/SS/L1 Prep Height", 5);

    public static final LoggedTunableNumber L2_CORAL_SCORE_HEIGHT_IN =
        new LoggedTunableNumber("Elevator/SS/L2 Score Height", 13);

    public static final LoggedTunableNumber L2_CORAL_PREP_HEIGHT_IN =
        new LoggedTunableNumber("Elevator/SS/L2 Prep Height", 13);

    public static final LoggedTunableNumber L3_CORAL_SCORE_HEIGHT_IN =
        new LoggedTunableNumber("Elevator/SS/L3 Score Height", 27);

    public static final LoggedTunableNumber L3_CORAL_PREP_HEIGHT_IN =
        new LoggedTunableNumber("Elevator/SS/L3 Prep Height", 27);

    public static final LoggedTunableNumber PROCESSOR_SCORE_HEIGHT_IN =
        new LoggedTunableNumber("Elevator/SS/Proc Score Height", 2);
  }

  public class Pivot {
    public static final LoggedTunableNumber SOURCE_CORAL_INTAKE_ANGLE_DEG =
        new LoggedTunableNumber("Pivot/SS/Source Intake Angle", 18);

    public static final LoggedTunableNumber L1_ALGAE_GRAB_DEG =
        new LoggedTunableNumber("Pivot/SS/L1 Algae Grab Angle", 150);
    public static final LoggedTunableNumber L3_ALGAE_GRAB_DEG =
        new LoggedTunableNumber("Pivot/SS/L3 Algae Grab Angle", 150);

    public static final LoggedTunableNumber L1_CORAL_SCORE_ANGLE_DEG =
        new LoggedTunableNumber("Pivot/SS/L1 Score Angle", 35);
    public static final LoggedTunableNumber L1_CORAL_PREP_ANGLE_DEG =
        new LoggedTunableNumber("Pivot/SS/L1 Prep Angle", 35);

    public static final LoggedTunableNumber L2_CORAL_SCORE_ANGLE_DEG =
        new LoggedTunableNumber("Pivot/SS/L2 Score Angle", 35);

    public static final LoggedTunableNumber L2_CORAL_PREP_ANGLE_DEG =
        new LoggedTunableNumber("Pivot/SS/L2 Prep Angle", 35);

    public static final LoggedTunableNumber L3_CORAL_SCORE_ANGLE_DEG =
        new LoggedTunableNumber("Pivot/SS/L3 Score Angle", 35);

    public static final LoggedTunableNumber L3_CORAL_PREP_ANGLE_DEG =
        new LoggedTunableNumber("Pivot/SS/L3 Prep Angle", 35);

    public static final LoggedTunableNumber PROCESSOR_SCORE_ANGLE_DEG =
        new LoggedTunableNumber("Pivot/SS/Proc Score Angle", 150);
  }

  public class Roller {
    public static final LoggedTunableNumber L1_CORAL_SCORE_SPEED =
        new LoggedTunableNumber("Roller/SS/L1 Score Speed", 3000);

    public static final LoggedTunableNumber L2_CORAL_SCORE_SPEED =
        new LoggedTunableNumber("Roller/SS/L2 Score Speed", 6000);

    public static final LoggedTunableNumber L3_CORAL_SCORE_SPEED =
        new LoggedTunableNumber("Roller/SS/L3 Score Speed", 6000);

    public static final LoggedTunableNumber SOURCE_CORAL_INTAKE_SPEED =
        new LoggedTunableNumber("Roller/SS/Source Intake Speed", 3000);

    public static final LoggedTunableNumber L1_ALGAE_GRAB_SPEED =
        new LoggedTunableNumber("Roller/SS/L1 Algae Grab Speed", -2000);
    public static final LoggedTunableNumber L3_ALGAE_GRAB_SPEED =
        new LoggedTunableNumber("Roller/SS/L3 Algae Grab Speed", -2000);

    public static final DoubleSupplier PROCESSOR_SCORE_SPEED =
        new LoggedTunableNumber("Roller/SS/Proc Score Speed", 4000);

    public static final DoubleSupplier ALGAE_HOLD_SPEED =
        new LoggedTunableNumber("Roller/SS/Algae Hold Speed", -2000);

    public static final DoubleSupplier ALGAE_CURRENT_LIMIT =
        new LoggedTunableNumber("Algae Current Limit", 40);

    public static final DoubleSupplier CORAL_CURRENT_LIMIT =
        new LoggedTunableNumber("Coral Current Limit", 80);
  }
}
