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
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;
  public static final boolean tuningMode = false;

  public static final double massKG = Units.lbsToKilograms(150);
  public static final double momentOfInertiaKGPerM2 = 6.0;

  public final class DriveConstants {
    public static final double wheelCOF = 1.1;

    public final class AutoContants {
      public static final PIDController xTrajectoryController = new PIDController(10.0, 0.0, 0.0);
      public static final PIDController yTrajectoryController = new PIDController(10.0, 0.0, 0.0);
      public static final PIDController headingTrajectoryController = new PIDController(5, 0.0, 0);
    }

    public final class OTFConstants {
      public static final PIDConstants translationPID = new PIDConstants(5.0, 0.0, 0.0);
      public static final PIDConstants rotationPID = new PIDConstants(5.0, 0.0, 0.0);
    }

    public final class HeadingControllerConstants {
      public static final double ANGLE_MAX_VELOCITY = 8.0;
      public static final double ANGLE_MAX_ACCELERATION = 20.0;
      public static final ProfiledPIDController angleController =
          new ProfiledPIDController(
              5.0,
              0.0,
              0.4,
              new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
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
