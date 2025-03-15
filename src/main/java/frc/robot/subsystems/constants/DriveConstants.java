package frc.robot.subsystems.constants;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.util.ControlGains;
import frc.robot.util.LoggedTunableGains;
import frc.robot.util.LoggedTunableNumber;

public class DriveConstants {
  public static final double wheelCOF = 1.1;

  // positive values adjust the robots scoring pose to the right.
  public static final LoggedTunableNumber coralOffsetFromCenter =
      new LoggedTunableNumber(
          "Drive/coralOffsetFromCenter", edu.wpi.first.math.util.Units.inchesToMeters(3.5));
  public static final double driveBaseWidthWithBumpersMeters =
      edu.wpi.first.math.util.Units.inchesToMeters(28.25 + 9.0);

  public static final LoggedTunableGains scoreAssistGains =
      new LoggedTunableGains("ScoreAssist", new ControlGains().p(2).d(0).trapezoidal(0.1, 0, 0));

  public final class AutoConstants {
    public static final LoggedTunableGains xTrajectoryController =
        new LoggedTunableGains("xTraj", new ControlGains().p(2.0));
    public static final LoggedTunableGains yTrajectoryController =
        new LoggedTunableGains("yTraj", new ControlGains().p(2.0));
    public static final LoggedTunableGains headingTrajectoryController =
        new LoggedTunableGains("headingTraj", new ControlGains().p(1.5).d(0.0));
  }

  public final class OTFConstants {
    public static final PIDConstants translationPID =
        new ControlGains().p(10.0).createPathPlannerGains();
    public static final PIDConstants rotationPID =
        new ControlGains().p(10.0).createPathPlannerGains();
    public static final double translationTolerance = 0.01;
  }

  public final class HeadingControllerConstants {
    public static final double kMaxAngularVelocity = 8.0;
    public static final double kMaxAngularAcceleration = 20.0;

    public static final ProfiledPIDController joystickAngleController =
        new ControlGains()
            .p(5.0)
            .d(0.4)
            .trapezoidal(kMaxAngularVelocity, kMaxAngularAcceleration, 0)
            .createTrapezoidalPIDController();

    public static final LoggedTunableGains angleGains =
        new LoggedTunableGains(
            "Drive/Angle Controller",
            new ControlGains()
                .p(8.0)
                .d(0.4)
                .trapezoidal(kMaxAngularVelocity, kMaxAngularAcceleration, 0));
    public static final ProfiledPIDController angleController =
        new ControlGains()
            .p(8.0)
            .d(0.4)
            .trapezoidal(kMaxAngularVelocity, kMaxAngularAcceleration, 0)
            .createAngularTrapezoidalPIDController();
  }

  public static RobotConfig pathPlannerConfig;

  // Update from GUI Settings
  static {
    try {
      pathPlannerConfig = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
  }

  private static RobotConfig fromManualSettings() {
    return new RobotConfig(
        Constants.massKG,
        Constants.momentOfInertiaKGPerM2,
        new ModuleConfig(
            TunerConstants.kWheelRadius.in(Units.Meters),
            TunerConstants.kSpeedAt12Volts.in(Units.MetersPerSecond),
            wheelCOF,
            DCMotor.getKrakenX60(1),
            TunerConstants.kSlipCurrent.in(Units.Amps),
            1),
        new Translation2d[] {
          new Translation2d(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
          new Translation2d(
              TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
          new Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
          new Translation2d(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)
        });
  }
}
