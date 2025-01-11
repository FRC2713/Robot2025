package frc.robot.subsystems.pivot;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.constants.PivotConstants;

public class PivotSparks implements PivotIO {
  private SparkMax motor;
  private double targetDegrees;

  public PivotSparks() {
    this.motor = new SparkMax(PivotConstants.kCANId, MotorType.kBrushless);
    this.motor.configure(
        PivotConstants.createConfig(),
        ResetMode.kResetSafeParameters,
        PersistMode.kNoPersistParameters);
  }

  @Override
  public void setVoltage(double volts) {
    motor.setVoltage(volts);
  }

  @Override
  public void updateInputs(PivotInputs inputs) {
    inputs.velocityDPS = motor.getEncoder().getVelocity();
    inputs.voltage = motor.getAppliedOutput();
    inputs.isOn =
        Math.abs(Units.rotationsPerMinuteToRadiansPerSecond(motor.getEncoder().getVelocity()))
            > 0.005;
    inputs.angleDegrees = motor.getEncoder().getPosition() * Math.PI * 2;
    inputs.commandedAngle = targetDegrees;
  }

  @Override
  public void setTargetAngle(double degrees) {
    this.targetDegrees = degrees;
    motor.getClosedLoopController().setReference(degrees, ControlType.kMAXMotionPositionControl);
  }
}
