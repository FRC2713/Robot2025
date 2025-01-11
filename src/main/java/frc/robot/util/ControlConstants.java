package frc.robot.util;

import com.revrobotics.spark.config.ClosedLoopConfig;
import lombok.Getter;

public class ControlConstants {

  @Getter private double kP = 0;
  @Getter private double kI = 0;
  @Getter private double kD = 0;

  @Getter private double kS = 0;
  @Getter private double kV = 0;
  @Getter private double kA = 0;

  @Getter private double kFF = 0;

  @Getter private double kG = 0;

  public ControlConstants() {}

  public ControlConstants p(double kP) {
    this.kP = kP;
    return this;
  }

  public ControlConstants i(double kI) {
    this.kI = kI;
    return this;
  }

  public ControlConstants d(double kD) {
    this.kD = kD;
    return this;
  }

  public ControlConstants s(double kS) {
    this.kS = kS;
    return this;
  }

  public ControlConstants v(double kV) {
    this.kV = kV;
    return this;
  }

  public ControlConstants a(double kA) {
    this.kA = kA;
    return this;
  }

  public ControlConstants ff(double kFF) {
    this.kFF = kFF;
    return this;
  }

  public ControlConstants g(double kG) {
    this.kG = kG;
    return this;
  }

  public ClosedLoopConfig applyPID(ClosedLoopConfig config) {
    return config.p(this.getKP()).i(this.getKI()).d(this.getKD());
  }
}
