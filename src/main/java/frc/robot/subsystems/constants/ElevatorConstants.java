package frc.robot.subsystems.constants;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.util.ControlGains;
import frc.robot.util.LoggedTunablePID;
import frc.robot.util.RHRUtil;

public class ElevatorConstants {
  public static final int kLeftCANId = 61;
  public static final int kRightCANId = 62;

  public static final boolean kIsLeftInverted = false;
  public static final boolean kIsRightInverted = true;

  public static final int kStallCurrentLimit = 30; // amps, value from 2024
  public static final int kDisableCurrentLimit = 120; // amps, values from 2024

  public static final int kMaxCurrentLimit = 40;

  public static final double kGearReduction = 5.0;
  public static final double kCarriageMass = Units.lbsToKilograms(0.249377); // kg
  public static final double kDrumRadius = 1.0;
  public static final double kAcceptablePositionErrorInches = 2; // inches

  public static final double kRotationsToHeightConversion = (Math.PI * kDrumRadius * 2);

  public static final double kMinHeight = 0.0; // inches
  public static final double kMaxHeight = 44.7; //52.5;
  public static final double kInitialHeight = 0.0;

  public static final LoggedTunablePID PID_LEVEL_ONE =
      new LoggedTunablePID(
          "Elevator_L_ONE",
          new ControlGains()
              // PID
              .p(RHRUtil.modeDependentDouble(19., 0.1))
              .i(0.0)
              .d(RHRUtil.modeDependentDouble(1, 0.01))
              // FF
              .g(RHRUtil.modeDependentDouble(9, 0.0785))
              .v(0.0)
              .a(0.0)
              // Motion Magic
              .mmCruiseVelo(1.5),
          160,
          1600);

  public static final double LEVEL_TWO_HEIGHT_IN = 25.13427520950623;

  public static final LoggedTunablePID PID_LEVEL_TWO =
      new LoggedTunablePID(
          "Elevator_L_TWO",
          new ControlGains()
              // PID
              .p(RHRUtil.modeDependentDouble(19., 0.1))
              .i(0.0)
              .d(RHRUtil.modeDependentDouble(1, 0.01))
              // FF
              .g(RHRUtil.modeDependentDouble(10, 0.0785))
              .v(0.0)
              .a(0.0));

  public static final int mech2dWidth = 20;
  public static final Color8Bit mech2dColor = new Color8Bit(255, 255, 0);

  public static TalonFXConfiguration createKrakenConfig(boolean inverted) {
    var config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Feedback.SensorToMechanismRatio = ElevatorConstants.kGearReduction;
    config.TorqueCurrent.PeakForwardTorqueCurrent = ElevatorConstants.kMaxCurrentLimit;
    config.TorqueCurrent.PeakReverseTorqueCurrent = -ElevatorConstants.kMaxCurrentLimit;
    config.CurrentLimits.StatorCurrentLimit = ElevatorConstants.kMaxCurrentLimit;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.MotorOutput.Inverted =
        (inverted) ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive;

    // set slot 0 gains
    config.Slot0 = PID_LEVEL_ONE.toTalonFX();
    config.Slot1 = PID_LEVEL_TWO.toTalonFXS1();

    // set Motion Magic settings
    config.MotionMagic = PID_LEVEL_ONE.toMotionMagic();

    return config;
  }

  public static final double AT_TARGET_GIVE_INCHES = 1.0;
}
