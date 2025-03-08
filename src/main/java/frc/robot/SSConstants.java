package frc.robot;

import frc.robot.subsystems.constants.ClimberConstants;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;

public class SSConstants {
  public class Elevator {
    public static final LoggedTunableNumber SOURCE_CORAL_INTAKE_HEIGHT_IN =
        new LoggedTunableNumber("Elevator/SS/Source Coral Intake Height", 6);

    public static final LoggedTunableNumber L1_HEIGHT_IN =
        new LoggedTunableNumber("Elevator/SS/L1 Height", 0);

    public static final LoggedTunableNumber L2_HEIGHT_IN =
        new LoggedTunableNumber("Elevator/SS/L2 Height", 2);

    public static final LoggedTunableNumber L3_HEIGHT_IN =
        new LoggedTunableNumber("Elevator/SS/L3 Height", 16);

    public static final LoggedTunableNumber L4_HEIGHT_IN =
        new LoggedTunableNumber("Elevator/SS/L4 Height", 24);

    public static final LoggedTunableNumber PROCESSOR_PREP_HEIGHT_IN =
        new LoggedTunableNumber("Elevator/SS/Proc Prep Height", 10);

    public static final LoggedTunableNumber PROCESSOR_SCORE_HEIGHT_IN =
        new LoggedTunableNumber("Elevator/SS/Proc Score Height", 10);

    public static final LoggedTunableNumber BARGE_HEIGHT_IN =
        new LoggedTunableNumber("Elevator/SS/Barge Height", 26);
  }

  public class Shoulder {
    public static final LoggedTunableNumber SOURCE_CORAL_INTAKE_ANGLE_DEG =
        new LoggedTunableNumber("Shoulder/SS/Source Intake Angle", -80);

    public static final LoggedTunableNumber L1_ANGLE_DEG =
        new LoggedTunableNumber("Shoulder/SS/L1 Angle", -135);

    public static final LoggedTunableNumber L2_ANGLE_DEG =
        new LoggedTunableNumber("Shoulder/SS/L2 Angle", -150);

    public static final LoggedTunableNumber L3_ANGLE_DEG =
        new LoggedTunableNumber("Shoulder/SS/L3 Angle", -150);

    public static final LoggedTunableNumber L4_ANGLE_DEG =
        new LoggedTunableNumber("Shoulder/SS/L4 Prep Angle", 130);

    public static final LoggedTunableNumber PROCESSOR_SCORE_ANGLE_DEG =
        new LoggedTunableNumber("Shoulder/SS/Proc Score Angle", -30);

    public static final LoggedTunableNumber PREP_CLIMB_ANGLE_DEGS =
        new LoggedTunableNumber("Shoulder/SS/Prep Climb Angle", -140);

    public static final LoggedTunableNumber CLIMB_ANGLE_DEGS =
        new LoggedTunableNumber("Shoulder/SS/Climb Angle", -147);

    public static final LoggedTunableNumber BARGE_ANGLE_DEGREES =
        new LoggedTunableNumber("Shoulder/SS/Barge Angle", 90);
  }

  public class Pivot {
    public static final LoggedTunableNumber SOURCE_CORAL_INTAKE_ANGLE_DEG =
        new LoggedTunableNumber("Pivot/SS/Source Intake Angle", 35);

    public static final LoggedTunableNumber L1_ANGLE_DEG =
        new LoggedTunableNumber("Pivot/SS/L1 Angle", 0);

    public static final LoggedTunableNumber L2_ANGLE_DEG =
        new LoggedTunableNumber("Pivot/SS/L2 Angle", 8);

    public static final LoggedTunableNumber L3_ANGLE_DEG =
        new LoggedTunableNumber("Pivot/SS/L3 Angle", 15);

    public static final LoggedTunableNumber L4_ANGLE_DEG =
        new LoggedTunableNumber("Pivot/SS/L4 Angle", 55);

    public static final LoggedTunableNumber BARGE_ANGLE_DEG =
        new LoggedTunableNumber("Pivot/SS/Barge Angle", -30);

    public static final LoggedTunableNumber PREP_CLIMB_ANGLE_DEGS =
        new LoggedTunableNumber("Pivot/SS/Prep Climb Angle", 13);

    public static final LoggedTunableNumber PROCESSOR_SCORE_ANGLE_DEG =
        new LoggedTunableNumber("Pivot/SS/Proc Score Angle", -180);

    public static final LoggedTunableNumber CLIMB_ANGLE_DEGS =
        new LoggedTunableNumber("Pivot/SS/Climb Angle", -111);
    ;

    public static final LoggedTunableNumber ALGAE_GRAB_ANGLE_DEGS =
        new LoggedTunableNumber("Pivot/SS/Algae Grab Angle", 35);

    public static final LoggedTunableNumber SAFE_ANGLE_DEGS =
        new LoggedTunableNumber("Pivot/SS/Safe Angle", 8);
  }

  public class Roller {
    public static final LoggedTunableNumber L1_CORAL_SCORE_SPEED =
        new LoggedTunableNumber("Roller/SS/L1 Score Speed", 3000);

    public static final LoggedTunableNumber L2_PLUS_CORAL_SCORE_SPEED =
        new LoggedTunableNumber("Roller/SS/L 2+ Score Speed", 4000);

    public static final LoggedTunableNumber SOURCE_CORAL_INTAKE_SPEED =
        new LoggedTunableNumber("Roller/SS/Source Intake Speed", 4000);
  }

  public class AlgaeClaw {
    public static final LoggedTunableNumber ALGAE_GRAB_SPEED =
        new LoggedTunableNumber("AlgaeClaw/SS/Algae Grab Speed", 4000);

    public static final DoubleSupplier PROCESSOR_SCORE_SPEED =
        new LoggedTunableNumber("AlgaeClaw/SS/Proc Score Speed", -6000);

    public static final DoubleSupplier BARGE_SCORE_SPEED =
        new LoggedTunableNumber("AlgaeClaw/SS/Barge Score Speed", -4000);

    public static final DoubleSupplier ALGAE_HOLD_SPEED =
        new LoggedTunableNumber("AlgaeClaw/SS/Algae Hold Speed", 2000);

    public static final DoubleSupplier ALGAE_DETECTED_CURRENT_LIMIT =
        new LoggedTunableNumber("AlgaeClaw/SS/Algae Detected Current Limit", 30);
  }

  public class Climber {
    public static final LoggedTunableNumber INP_TO_VOLTS =
        new LoggedTunableNumber("Climber/SS/Inputs to Volts", 10);

    public static final LoggedTunableNumber SERVO_POS_ON =
        new LoggedTunableNumber("Climber/SS/Servo Pos On", 0.5);
    public static final LoggedTunableNumber SERVO_POS_OFF =
        new LoggedTunableNumber("Climber/SS/Servo Pos Off", 0.0);

    public static final LoggedTunableNumber MIN_ANGLE_CLIMBING =
        new LoggedTunableNumber("Climber/SS/Min Angle Climbing", 0.0);
    public static final LoggedTunableNumber MAX_ANGLE_CLIMBING =
        new LoggedTunableNumber("Climber/SS/Max Angle Climbing", ClimberConstants.kMaxAngle);

    public static final LoggedTunableNumber CLEARANCE_ANGLE =
        new LoggedTunableNumber("Climber/SS/Clearance Angle", 180.0);
  }

  public class Auto {
    public static final LoggedTunableNumber INTAKE_DELAY =
        new LoggedTunableNumber("Auto/Intake Delay", 1);
    public static final LoggedTunableNumber L4_SCORE_DELAY =
        new LoggedTunableNumber("Auto/Intake Delay", 0.2);
  }
}
