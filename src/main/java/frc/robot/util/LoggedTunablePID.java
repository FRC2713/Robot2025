package frc.robot.util;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants;

public class LoggedTunablePID {
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

  public LoggedTunablePID(String subsystem, ControlGains PID) {
    this(subsystem, PID, 0.0, 0.0);
  }

  public LoggedTunablePID(
      String subsystem, ControlGains PID, double MotionMagicAcceleration, double MotionMagicJerk) {
    kP = new LoggedTunableNumber(subsystem + "/P", PID.getKP());
    kI = new LoggedTunableNumber(subsystem + "/I", PID.getKI());
    kD = new LoggedTunableNumber(subsystem + "/D", PID.getKD());
    kG = new LoggedTunableNumber(subsystem + "/G", PID.getKG());
    kS = new LoggedTunableNumber(subsystem + "/S", PID.getKS());
    kV = new LoggedTunableNumber(subsystem + "/V", PID.getKV());
    kA = new LoggedTunableNumber(subsystem + "/A", PID.getKA());
    MotionMagicCruiseVelocity =
        new LoggedTunableNumber(subsystem + "/CruiseVelocity", PID.getKMMCruiseVelo());
    this.MotionMagicAcceleration =
        new LoggedTunableNumber(subsystem + "/Acceleration", MotionMagicAcceleration);
    this.MotionMagicJerk = new LoggedTunableNumber(subsystem + "/Jerk", MotionMagicJerk);
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
        || MotionMagicJerk.hasChanged(id);
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
  public Slot0Configs toTaloxFX() {
    var slot0Configs = new Slot0Configs();

    slot0Configs.kP = getKP();
    slot0Configs.kI = getKI();
    slot0Configs.kD = getKD();

    slot0Configs.kG = getKG();
    slot0Configs.kS = getKS();
    slot0Configs.kV = getKV();
    slot0Configs.kA = getKA();

    return slot0Configs;
  }

  public MotionMagicConfigs toMotionMagic() {
    var motionMagicConfigs = new MotionMagicConfigs();

    motionMagicConfigs.MotionMagicCruiseVelocity = MotionMagicCruiseVelocity.getAsDouble();
    motionMagicConfigs.MotionMagicAcceleration = MotionMagicAcceleration.getAsDouble();
    motionMagicConfigs.MotionMagicJerk = MotionMagicJerk.getAsDouble();

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

  public ElevatorFeedforward createElevatorFF() {
    return new ElevatorFeedforward(this.getKS(), this.getKG(), this.getKV(), this.getKA());
  }
}
