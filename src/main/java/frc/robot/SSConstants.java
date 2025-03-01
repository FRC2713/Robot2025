package frc.robot;

import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;

public class SSConstants {
  public class Elevator {
    public static final LoggedTunableNumber SOURCE_CORAL_INTAKE_HEIGHT_IN =
        new LoggedTunableNumber("Elevator/SS/Source Coral Intake Height", 0);

    public static final LoggedTunableNumber L1_ALGAE_GRAB_HEIGHT_IN =
        new LoggedTunableNumber("Elevator/SS/L1 Algae Grab Height", 0);

    public static final LoggedTunableNumber L3_ALGAE_GRAB_HEIGHT_IN =
        new LoggedTunableNumber("Elevator/SS/L3 Algae Grab Height", 0);

    public static final LoggedTunableNumber L1_CORAL_SCORE_HEIGHT_IN =
        new LoggedTunableNumber("Elevator/SS/L1 Score Height", 0);
    public static final LoggedTunableNumber L1_CORAL_PREP_HEIGHT_IN =
        new LoggedTunableNumber("Elevator/SS/L1 Prep Height", 0);

    public static final LoggedTunableNumber L2_CORAL_SCORE_HEIGHT_IN =
        new LoggedTunableNumber("Elevator/SS/L2 Score Height", 0);

    public static final LoggedTunableNumber L2_CORAL_PREP_HEIGHT_IN =
        new LoggedTunableNumber("Elevator/SS/L2 Prep Height", 0);

    public static final LoggedTunableNumber L3_CORAL_SCORE_HEIGHT_IN =
        new LoggedTunableNumber("Elevator/SS/L3 Score Height", 0);

    public static final LoggedTunableNumber L3_CORAL_PREP_HEIGHT_IN =
        new LoggedTunableNumber("Elevator/SS/L3 Prep Height", 0);

    public static final LoggedTunableNumber L4_CORAL_SCORE_HEIGHT_IN =
        new LoggedTunableNumber("Elevator/SS/L4 Score Angle", 20);
    public static final LoggedTunableNumber L4_CORAL_PREP_HEIGHT_IN =
        new LoggedTunableNumber("Elevator/SS/L4 Prep Angle", 20);

    public static final LoggedTunableNumber PROCESSOR_SCORE_HEIGHT_IN =
        new LoggedTunableNumber("Elevator/SS/Proc Score Height", 10);
  }

  public class Shoulder {
    public static final LoggedTunableNumber SOURCE_CORAL_INTAKE_ANGLE_DEG =
        new LoggedTunableNumber("Shoulder/SS/Source Intake Angle", -90);
    public static final LoggedTunableNumber L1_ALGAE_GRAB_ANGLE_DEG =
        new LoggedTunableNumber("Shoulder/SS/L1 Algae Grab Angle", -200);
    public static final LoggedTunableNumber L3_ALGAE_GRAB_ANGLE_DEG =
        new LoggedTunableNumber("Shoulder/SS/L3 Algae Grab Angle", -210);

    public static final LoggedTunableNumber L1_CORAL_SCORE_ANGLE_DEG =
        new LoggedTunableNumber("Shoulder/SS/L1 Score Angle", -130);
    public static final LoggedTunableNumber L1_CORAL_PREP_ANGLE_DEG =
        new LoggedTunableNumber("Shoulder/SS/L1 Prep Angle", -130);

    public static final LoggedTunableNumber L2_CORAL_SCORE_ANGLE_DEG =
        new LoggedTunableNumber("Shoulder/SS/L2 Score Angle", -160);

    public static final LoggedTunableNumber L2_CORAL_PREP_ANGLE_DEG =
        new LoggedTunableNumber("Shoulder/SS/L2 Prep Angle", -160);

    public static final LoggedTunableNumber L3_CORAL_SCORE_ANGLE_DEG =
        new LoggedTunableNumber("Shoulder/SS/L3 Score Angle", -200);

    public static final LoggedTunableNumber L3_CORAL_PREP_ANGLE_DEG =
        new LoggedTunableNumber("Shoulder/SS/L3 Prep Angle", -200);

    public static final LoggedTunableNumber L4_CORAL_SCORE_ANGLE_DEG =
        new LoggedTunableNumber("Shoulder/SS/L4 Score Angle", -250);
    public static final LoggedTunableNumber L4_CORAL_PREP_ANGLE_DEG =
        new LoggedTunableNumber("Shoulder/SS/L4 Prep Angle", -250);

    public static final LoggedTunableNumber PROCESSOR_SCORE_ANGLE_DEG =
        new LoggedTunableNumber("Shoulder/SS/Proc Score Angle", -30);
  }

  public class Pivot {
    public static final LoggedTunableNumber SOURCE_CORAL_INTAKE_ANGLE_DEG =
        new LoggedTunableNumber("Pivot/SS/Source Intake Angle", 35);

    public static final LoggedTunableNumber L1_ALGAE_GRAB_DEG =
        new LoggedTunableNumber("Pivot/SS/L1 Algae Grab Angle", 35);
    public static final LoggedTunableNumber L3_ALGAE_GRAB_DEG =
        new LoggedTunableNumber("Pivot/SS/L3 Algae Grab Angle", 35);

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

    public static final LoggedTunableNumber L4_CORAL_SCORE_ANGLE_DEG =
        new LoggedTunableNumber("Pivot/SS/L4 Score Angle", 45);
    public static final LoggedTunableNumber L4_CORAL_PREP_ANGLE_DEG =
        new LoggedTunableNumber("Pivot/SS/L4 Prep Angle", 45);

    public static final LoggedTunableNumber PROCESSOR_SCORE_ANGLE_DEG =
        new LoggedTunableNumber("Pivot/SS/Proc Score Angle", -180);
  }

  public class Roller {
    public static final LoggedTunableNumber L1_CORAL_SCORE_SPEED =
        new LoggedTunableNumber("Roller/SS/L1 Score Speed", 3000);

    public static final LoggedTunableNumber L2_PLUS_CORAL_SCORE_SPEED =
        new LoggedTunableNumber("Roller/SS/L 2+ Score Speed", 6000);

    public static final LoggedTunableNumber SOURCE_CORAL_INTAKE_SPEED =
        new LoggedTunableNumber("Roller/SS/Source Intake Speed", 6000);
  }

  public class AlgaeClaw {
    public static final LoggedTunableNumber ALGAE_GRAB_SPEED =
        new LoggedTunableNumber("AlgaeClaw/SS/Algae Grab Speed", 3000);

    public static final DoubleSupplier PROCESSOR_SCORE_SPEED =
        new LoggedTunableNumber("AlgaeClaw/SS/Proc Score Speed", -3000);

    public static final DoubleSupplier ALGAE_HOLD_SPEED =
        new LoggedTunableNumber("AlgaeClaw/SS/Algae Hold Speed", 2000);

    public static final DoubleSupplier ALGAE_DETECTED_CURRENT_LIMIT =
        new LoggedTunableNumber("AlgaeClaw/SS/Algae Detected Current Limit", 40);
  }
}
