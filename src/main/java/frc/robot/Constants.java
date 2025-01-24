// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.util.ControlGains;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;
  public static final boolean tuningMode = false;
  public static final double simulationRate = 0.020;

  public static final double massKG = Units.lbsToKilograms(150);
  public static final double momentOfInertiaKGPerM2 = 6.0;

  public final class DriveConstants {
    public static final double wheelCOF = 1.1;

    public final class AutoContants {
      public static final PIDController xTrajectoryController =
          new ControlGains().p(10.0).createPIDController();
      public static final PIDController yTrajectoryController =
          new ControlGains().p(10.0).createPIDController();
      public static final PIDController headingTrajectoryController =
          new ControlGains().p(5.0).createPIDController();
    }

    public final class OTFConstants {
      public static final PIDConstants translationPID =
          new ControlGains().p(5.0).createPathPlannerGains();
      public static final PIDConstants rotationPID =
          new ControlGains().p(5.0).createPathPlannerGains();
    }

    public final class HeadingControllerConstants {
      public static final double kMaxAngularVelocity = 8.0;
      public static final double kMaxAngularAcceleration = 20.0;
      public static final ProfiledPIDController angleController =
          new ControlGains()
              .p(5.0)
              .d(0.4)
              .trapezoidal(kMaxAngularVelocity, kMaxAngularAcceleration)
              .createTrapezoidalPIDController();
    }

    public static RobotConfig pathPlannerConfig;
    // new RobotConfig(
    //     Constants.massKG,
    //     Constants.momentOfInertiaKGPerM2,
    //     new ModuleConfig(
    //         TunerConstants.kWheelRadius.in(Meters),
    //         TunerConstants.kSpeedAt12Volts.in(MetersPerSecond),
    //         wheelCOF,
    //         DCMotor.getKrakenX60(1),
    //         TunerConstants.kSlipCurrent.in(Amps),
    //         1),
    //     new Translation2d[] {
    //       new Translation2d(
    //           TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
    //       new Translation2d(
    //           TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
    //       new Translation2d(
    //           TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
    //       new Translation2d(
    //           TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)
    //     });

    // Update from GUI Settings
    static {
      try {
        pathPlannerConfig = RobotConfig.fromGUISettings();
      } catch (Exception e) {
        // TODO Auto-generated catch block
        e.printStackTrace();
      }
    }
  }

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }
}
