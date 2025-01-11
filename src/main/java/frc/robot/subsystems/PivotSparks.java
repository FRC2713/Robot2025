package frc.robot.subsystems;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;


import edu.wpi.first.math.util.Units;

public class PivotSparks implements PivotIO {
  private SparkMax motor;

  public PivotSparks() {
    this.motor = new SparkMax(98, MotorType.kBrushless);
    this.motor.getEncoder().setPositionConversionFactor(1.0 / 225.0 * 1);
  }

  @Override
  public void setVoltage(double Volts) {
    motor.setVoltage(Volts);
  }

  @Override
  public void updateInputs(PivotInputs inputs) {
    inputs.velocityDPS = motor.getEncoder().getVelocity();
    inputs.voltage = motor.getAppliedOutput();
    inputs.isOn =
        Math.abs(Units.rotationsPerMinuteToRadiansPerSecond(motor.getEncoder().getVelocity()))
            > 0.005;
    inputs.angleDegrees = motor.getEncoder().getPosition() * Math.PI * 2;
  }
  

@Override
public void setTargetAngle(double degrees) {
  motor.getClosedLoopController().setReference(degrees, ControlType.kMAXMotionPositionControl);
}
}
