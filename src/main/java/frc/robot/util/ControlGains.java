package frc.robot.util;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.pathplanner.lib.config.PIDConstants;
import com.revrobotics.spark.config.ClosedLoopConfig;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import lombok.Getter;

public class ControlGains {

  @Getter private double kP = 0;
  @Getter private double kI = 0;
  @Getter private double kD = 0;

  @Getter private double kS = 0;
  @Getter private double kV = 0;
  @Getter private double kA = 0;
  @Getter private double kG = 0;

  @Getter private double kFF = 0;

  @Getter private double kTrapezoidalMaxVelocity;
  @Getter private double kTrapezoidalMaxAcceleration;
  @Getter private double kTrapezoidalMaxJerk;
  @Getter private double kExponential_kV;
  @Getter private double kExponential_kA;

  public ControlGains() {}

  public ControlGains p(double kP) {
    this.kP = kP;
    return this;
  }

  public ControlGains i(double kI) {
    this.kI = kI;
    return this;
  }

  public ControlGains d(double kD) {
    this.kD = kD;
    return this;
  }

  public ControlGains s(double kS) {
    this.kS = kS;
    return this;
  }

  public ControlGains v(double kV) {
    this.kV = kV;
    return this;
  }

  public ControlGains a(double kA) {
    this.kA = kA;
    return this;
  }

  public ControlGains ff(double kFF) {
    this.kFF = kFF;
    return this;
  }

  public ControlGains g(double kG) {
    this.kG = kG;
    return this;
  }

  public ControlGains trapezoidal(double kMaxVel, double kMaxAccel, double kMaxJerk) {
    this.kTrapezoidalMaxVelocity = kMaxVel;
    this.kTrapezoidalMaxAcceleration = kMaxAccel;
    this.kTrapezoidalMaxJerk = kMaxJerk;
    return this;
  }

  public ControlGains maxTrapezoidalVelocity(double velo) {
    this.kTrapezoidalMaxVelocity = velo;
    return this;
  }

  public ControlGains maxTrapezoidalAcceleration(double accel) {
    this.kTrapezoidalMaxAcceleration = accel;
    return this;
  }

  public ControlGains maxTrapezoidalJerk(double jerk) {
    this.kTrapezoidalMaxJerk = jerk;
    return this;
  }

  public ControlGains exponential(double kV, double kA) {
    this.kExponential_kV = kV;
    this.kExponential_kA = kA;
    return this;
  }

  public ControlGains expo_kV(double kV) {
    this.kExponential_kV = kV;
    return this;
  }

  public ControlGains expo_kA(double kA) {
    this.kExponential_kA = kA;
    return this;
  }

  public PIDConstants createPathPlannerGains() {
    return new PIDConstants(this.getKP(), this.getKI(), this.getKD());
  }

  public ClosedLoopConfig applyPID(ClosedLoopConfig config) {
    return config.p(this.getKP()).i(this.getKI()).d(this.getKD());
  }

  public PIDController createPIDController() {
    return new PIDController(this.getKP(), this.getKI(), this.getKD());
  }

  public ProfiledPIDController createTrapezoidalPIDController() {
    return new ProfiledPIDController(
        this.getKP(),
        this.getKI(),
        this.getKD(),
        new TrapezoidProfile.Constraints(
            this.getKTrapezoidalMaxVelocity(), this.getKTrapezoidalMaxAcceleration()));
  }

  public ElevatorFeedforward createElevatorFF() {
    return new ElevatorFeedforward(this.getKS(), this.getKG(), this.getKV(), this.getKA());
  }

  public MotionMagicConfigs createMMConfigs() {
    var configs = new MotionMagicConfigs();
    configs.MotionMagicCruiseVelocity = this.kTrapezoidalMaxVelocity;
    configs.MotionMagicExpo_kA = this.kExponential_kA;
    configs.MotionMagicExpo_kV = this.kExponential_kV;

    return configs;
  }
}
