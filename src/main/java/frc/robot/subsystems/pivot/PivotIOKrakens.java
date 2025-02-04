package frc.robot.subsystems.pivot;

import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.constants.PivotConstants;
import frc.robot.subsystems.pivot.PivotIO.PivotInputs;
import frc.robot.util.PhoenixUtil;

public class PivotIOKrakens {
  private final TalonFX motor;
  private double targetDegrees;
  private final MotionMagicExpoTorqueCurrentFOC angleRequest =
      new MotionMagicExpoTorqueCurrentFOC(0);

  public PivotIOKrakens() {
    this.motor = new TalonFX(PivotConstants.kCANId);
    var config = PivotConstants.createKrakenConfig();
    PhoenixUtil.tryUntilOk(5, () -> motor.getConfigurator().apply(config, 0.25));
    PhoenixUtil.tryUntilOk(5, () -> motor.setPosition(PivotConstants.kInitialAngle, 0.25));
  }

  public void setVoltage(double volts) {
    motor.setVoltage(volts);
  }

  public void setTargetAngle(double degrees) {
    this.targetDegrees = degrees;
    motor.setControl(angleRequest.withPosition(degrees));
  }

  public void updateInputs(PivotInputs inputs) {
    inputs.velocityDPS = motor.getVelocity().getValueAsDouble();
    inputs.voltage = motor.getMotorVoltage().getValueAsDouble();
    inputs.isOn =
        Math.abs(Units.rotationsPerMinuteToRadiansPerSecond(motor.getVelocity().getValueAsDouble()))
            > 0.005;
    inputs.angleDegrees = motor.getPosition().getValueAsDouble() * Math.PI * 2;
    inputs.commandedAngle = targetDegrees;
  }
}
