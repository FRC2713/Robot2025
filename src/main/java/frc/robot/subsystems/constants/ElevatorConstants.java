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

  public static final double kGearReduction = 9.0;
  public static final double kCarriageMass = Units.lbsToKilograms(0.249377); // kg
  public static final double kDrumRadius = 1.0;
  public static final double kAcceptablePositionErrorInches = 2; // inches

  public static final double kRotationsToHeightConversion = (1 / 25.0 * Math.PI * kDrumRadius);

  public static final double kMinHeight = 0.0; // inches
  public static final double kMaxHeight = 52.5;
  public static final double kInitialHeight = 0.0;

  public static final double kP = RHRUtil.modeDependentDouble(0., 0.1);
  public static final double kI = 0.0;
  public static final double kD = RHRUtil.modeDependentDouble(0, 0.01);

  public static final double kG = RHRUtil.modeDependentDouble(0, 0.0785);
  public static final double kV = 0.0;
  public static final double kA = 0.0;

  public static final double KTrapezoidalMaxVelocity = 10.0;
  public static final double KTrapezoidalMaxAcceleration = 10.0;

  public static final LoggedTunablePID PID =
      new LoggedTunablePID(
          "Elevator",
          new ControlGains()
              // PID
              .p(kP)
              .i(kI)
              .d(kD)
              // FF
              .g(kG)
              .v(kV)
              .a(kA)
              // Motion Magic
              .mmCruiseVelo(0),
          160,
          1600);

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
    config.Slot0 = PID.toTalonFX();

    // set Motion Magic settings
    config.MotionMagic = PID.toMotionMagic();

    return config;
  }

  public static final double AT_TARGET_GIVE_INCHES = 1.0;
}
