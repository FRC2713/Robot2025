package frc.robot.subsystems.constants;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.util.ControlGains;
import frc.robot.util.LoggedTunablePID;
import frc.robot.util.RHRUtil;

public class ShoulderConstants {
  public static final int kCANId = 17;
  public static final int kEncoderCANId = 18;
  public static final boolean kInverted = false;

  public static final double kGearing = (36.0 / 12.0) * 25;
  public static final double kLength = Units.inchesToMeters(18);
  public static final double kMass = 7.094328; // kg
  public static final double kHeight = Units.inchesToMeters(24);

  public static final double kMinAngleRad = Units.degreesToRadians(-280);
  public static final double kMaxAngleRad = Units.degreesToRadians(100);

  public static final double kInitialAngleRad = Units.degreesToRadians(-90);
  public static final double kRampAngleRad = Units.degreesToRadians(30);

  // transform from elevator origin to shoulder origin
  public static final Transform3d kInitialTransform =
      new Transform3d(
          0.0,
          Units.inchesToMeters(9.784055),
          Units.inchesToMeters(34.75),
          new Rotation3d(0, 0, 0));

  // pose in robot frame
  public static final Pose3d kInitialPose =
      ElevatorConstants.kInitialPose.transformBy(kInitialTransform);

  public static final int kStallCurrentLimit = 30; // amps
  public static final int kStatorCurrentLimit = 100; // also amps
  public static final double kMaxAngularVelocity = 5; // RPM

  public static final double kP = RHRUtil.modeDependentDouble(350., 10); // output/rotation
  public static final double kI = 0.0; // Integral of kP
  public static final double kD = RHRUtil.modeDependentDouble(20., 0); // output/error in velocity

  public static final double kG = RHRUtil.modeDependentDouble(15., 0.381);
  public static final double kV = 0.0; // kV * rev/s = volts
  public static final double kA = 0.0;
  public static final double kS = RHRUtil.modeDependentDouble(1.5, 0.); // Volts

  public static final double kTrapezoidalMaxVelocity = 3;
  public static final double kTrapezoidalMaxAcceleration = 30;
  public static final double kTrapezoidalMaxJerk = 300;

  public static final LoggedTunablePID PID =
      new LoggedTunablePID(
          "Shoulder",
          new ControlGains()
              // PID
              .p(kP)
              .i(kI)
              .d(kD)
              // FF
              .g(kG)
              .s(kS)
              .v(kV)
              .a(kA)
              // Motion Magic
              .mmCruiseVelo(1),
          80,
          1600);

  public static final double kAbsoluteEncoderOffset = Units.degreesToRotations(21);

  public static final int mech2dWidth = 10;
  public static final Color8Bit mech2dColor = new Color8Bit(0, 255, 0);

  public static TalonFXConfiguration createKrakenConfig() {
    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Feedback.SensorToMechanismRatio = ShoulderConstants.kGearing;
    config.Feedback.RotorToSensorRatio = 1;
    config.TorqueCurrent.PeakForwardTorqueCurrent = ShoulderConstants.kStallCurrentLimit;
    config.TorqueCurrent.PeakReverseTorqueCurrent = -ShoulderConstants.kStallCurrentLimit;
    config.CurrentLimits.StatorCurrentLimit = ShoulderConstants.kStatorCurrentLimit;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.MotorOutput.Inverted =
        (ShoulderConstants.kInverted)
            ? InvertedValue.CounterClockwise_Positive
            : InvertedValue.Clockwise_Positive;

    config.Slot0 = PID.toTalonFX(GravityTypeValue.Arm_Cosine);
    // config.MotionMagic = PID.toMotionMagic();

    config.MotionMagic.MotionMagicCruiseVelocity = .05;
    config.MotionMagic.MotionMagicAcceleration = kTrapezoidalMaxAcceleration;
    config.MotionMagic.MotionMagicJerk = kTrapezoidalMaxJerk;
    config.MotionMagic.MotionMagicExpo_kV = 6.4;
    config.MotionMagic.MotionMagicExpo_kA = 0.1;
    return config;
  }

  public static CANcoderConfiguration createCANcoderConfiguration() {
    var config = new CANcoderConfiguration();
    config.MagnetSensor.MagnetOffset = kAbsoluteEncoderOffset;

    return config;
  }

  public static final double AT_TARGET_GIVE_DEGS = 2;
}
