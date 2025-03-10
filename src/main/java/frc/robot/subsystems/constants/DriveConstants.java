package frc.robot.subsystems.constants;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.util.ControlGains;

public class DriveConstants {
  public static final double wheelCOF = 1.1;
  public static final double driveBaseWidthWithBumpersMeters =
      edu.wpi.first.math.util.Units.inchesToMeters(28.25 + 7.0);

  public final class AutoConstants {
    public static final PIDController xTrajectoryController =
        new ControlGains().p(10.0).createPIDController();
    public static final PIDController yTrajectoryController =
        new ControlGains().p(10.0).createPIDController();
    public static final PIDController headingTrajectoryController =
        new ControlGains().p(5.0).createPIDController();
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

    public static final ProfiledPIDController angleController =
        new ControlGains()
            .p(5.0)
            .d(0.4)
            .trapezoidal(kMaxAngularVelocity, kMaxAngularAcceleration, 0)
            .createTrapezoidalPIDController();
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
