package frc.robot.util;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants;

public class LoggedTunableGains {
  public final LoggedTunableNumber kP;
  public final LoggedTunableNumber kI;
  public final LoggedTunableNumber kD;

  public final LoggedTunableNumber kG;
  public final LoggedTunableNumber kS;
  public final LoggedTunableNumber kV;
  public final LoggedTunableNumber kA;

  public final LoggedTunableNumber MotionMagicCruiseVelocity;
  public final LoggedTunableNumber MotionMagicAcceleration;
  public final LoggedTunableNumber MotionMagicJerk;
  public final LoggedTunableNumber MotionMagicExpo_kV;
  public final LoggedTunableNumber MotionMagicExpo_kA;
  private String name;

  public LoggedTunableGains(String subsystem, ControlGains PID) {
    this.name = subsystem;
    kP = new LoggedTunableNumber(subsystem + "/PID/P", PID.getKP());
    kI = new LoggedTunableNumber(subsystem + "/PID/I", PID.getKI());
    kD = new LoggedTunableNumber(subsystem + "/PID/D", PID.getKD());
    kG = new LoggedTunableNumber(subsystem + "/PID/G", PID.getKG());
    kS = new LoggedTunableNumber(subsystem + "/PID/S", PID.getKS());
    kV = new LoggedTunableNumber(subsystem + "/PID/V", PID.getKV());
    kA = new LoggedTunableNumber(subsystem + "/PID/A", PID.getKA());
    MotionMagicCruiseVelocity =
        new LoggedTunableNumber(
            subsystem + "/PID/CruiseVelocity", PID.getKTrapezoidalMaxVelocity());
    this.MotionMagicAcceleration =
        new LoggedTunableNumber(
            subsystem + "/PID/Acceleration", PID.getKTrapezoidalMaxAcceleration());
    this.MotionMagicJerk =
        new LoggedTunableNumber(subsystem + "/PID/Jerk", PID.getKTrapezoidalMaxJerk());

    this.MotionMagicExpo_kV =
        new LoggedTunableNumber(subsystem + "/PID/Expo_kV", PID.getKExponential_kV());
    this.MotionMagicExpo_kA =
        new LoggedTunableNumber(subsystem + "/PID/Expo_kA", PID.getKExponential_kA());
  }

  public boolean hasChanged(int id) {
    if (!Constants.tuningMode) {
      return false;
    }
    return kP.hasChanged(id)
        || kI.hasChanged(id)
        || kD.hasChanged(id)
        || kG.hasChanged(id)
        || kS.hasChanged(id)
        || kV.hasChanged(id)
        || kA.hasChanged(id)
        || MotionMagicCruiseVelocity.hasChanged(id)
        || MotionMagicAcceleration.hasChanged(id)
        || MotionMagicJerk.hasChanged(id)
        || MotionMagicExpo_kV.hasChanged(id)
        || MotionMagicExpo_kA.hasChanged(id);
  }

  public double getKP() {
    return kP.getAsDouble();
  }

  public double getKI() {
    return kI.getAsDouble();
  }

  public double getKD() {
    return kD.getAsDouble();
  }

  public double getKG() {
    return kG.getAsDouble();
  }

  public double getKS() {
    return kS.getAsDouble();
  }

  public double getKV() {
    return kV.getAsDouble();
  }

  public double getKA() {
    return kA.getAsDouble();
  }

  // IMPORTANT: Further settings (e.g. gravity type) may need to be added to Slot0Configs
  public Slot0Configs toTalonFX() {
    return toTalonFX(GravityTypeValue.Elevator_Static);
  }

  public Slot0Configs toTalonFX(GravityTypeValue gravityType) {
    var slot0Configs = new Slot0Configs();

    slot0Configs.GravityType = gravityType;
    slot0Configs.kP = getKP();
    slot0Configs.kI = getKI();
    slot0Configs.kD = getKD();

    slot0Configs.kG = getKG();
    slot0Configs.kS = getKS();
    slot0Configs.kV = getKV();
    slot0Configs.kA = getKA();

    return slot0Configs;
  }

  public Slot1Configs toTalonFXS1() {
    return toTalonFXS1(GravityTypeValue.Elevator_Static);
  }

  public Slot1Configs toTalonFXS1(GravityTypeValue gravityType) {
    var slot1Configs = new Slot1Configs();

    slot1Configs.GravityType = gravityType;
    slot1Configs.kP = getKP();
    slot1Configs.kI = getKI();
    slot1Configs.kD = getKD();

    slot1Configs.kG = getKG();
    slot1Configs.kS = getKS();
    slot1Configs.kV = getKV();
    slot1Configs.kA = getKA();

    return slot1Configs;
  }

  public MotionMagicConfigs getMotionMagicConfig() {
    var motionMagicConfigs = new MotionMagicConfigs();

    motionMagicConfigs.MotionMagicCruiseVelocity = MotionMagicCruiseVelocity.getAsDouble();
    motionMagicConfigs.MotionMagicAcceleration = MotionMagicAcceleration.getAsDouble();
    motionMagicConfigs.MotionMagicJerk = MotionMagicJerk.getAsDouble();
    motionMagicConfigs.MotionMagicExpo_kV = MotionMagicExpo_kV.getAsDouble();
    motionMagicConfigs.MotionMagicExpo_kA = MotionMagicExpo_kA.getAsDouble();

    return motionMagicConfigs;
  }

  public ProfiledPIDController createTrapezoidalPIDController(
      double KTrapezoidalMaxVelocity, double getKTrapezoidalMaxAcceleration) {
    return new ProfiledPIDController(
        this.getKP(),
        this.getKI(),
        this.getKD(),
        new TrapezoidProfile.Constraints(KTrapezoidalMaxVelocity, getKTrapezoidalMaxAcceleration));
  }

  public ProfiledPIDController createTrapezoidalPIDController() {
    return new ProfiledPIDController(
        this.getKP(),
        this.getKI(),
        this.getKD(),
        new TrapezoidProfile.Constraints(
            MotionMagicCruiseVelocity.getAsDouble(), MotionMagicAcceleration.getAsDouble()));
  }

  public ProfiledPIDController createAngularTrapezoidalPIDController() {
    var pid = createTrapezoidalPIDController();
    pid.enableContinuousInput(-Math.PI, Math.PI);
    return pid;
  }

  public PIDController createPIDController() {
    return new PIDController(this.getKP(), this.getKI(), this.getKD());
  }

  public PIDController createAngularPIDController() {
    var pid = new PIDController(this.getKP(), this.getKI(), this.getKD());
    pid.enableContinuousInput(-Math.PI, Math.PI);
    return pid;
  }

  public ArmFeedforward createArmFF() {
    return new ArmFeedforward(this.getKS(), this.getKG(), this.getKV(), this.getKA());
  }

  public ElevatorFeedforward createElevatorFF() {
    return new ElevatorFeedforward(this.getKS(), this.getKG(), this.getKV(), this.getKA());
  }

  public ControlGains toControlGains() {
    return new ControlGains()
        .p(this.getKP())
        .i(this.getKI())
        .d(this.getKD())
        .s(this.getKS())
        .g(this.getKG())
        .a(this.getKG())
        .v(this.getKV());
  }

  public LoggedTunableGains slowDown() {
    return new LoggedTunableGains(
        "Slow_" + name,
        new ControlGains()
            .p(this.getKP())
            .i(this.getKI())
            .d(this.getKD())
            .s(this.getKS())
            .g(this.getKG())
            .a(this.getKG())
            .v(this.getKV())
            .maxTrapezoidalVelocity(MotionMagicCruiseVelocity.getAsDouble() / 2)
            .maxTrapezoidalAcceleration(MotionMagicAcceleration.getAsDouble() / 2));
  }
}
