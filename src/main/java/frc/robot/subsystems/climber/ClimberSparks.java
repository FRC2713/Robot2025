package frc.robot.subsystems.climber;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Servo;
import frc.robot.subsystems.constants.ClimberConstants;
import frc.robot.util.LoggedTunableGains;

public class ClimberSparks implements ClimberIO {
  public SparkFlex motor = new SparkFlex(ClimberConstants.kCANId, MotorType.kBrushless);
  public Servo servo = new Servo(1);
  private double targetRPM;

  public ClimberSparks() {
    motor.configure(
        ClimberConstants.createSparkConfig(),
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    motor.getEncoder().setPosition(Units.degreesToRotations(ClimberConstants.kInitialAngle));
  }

  @Override
  public void updateInputs(ClimberInputs inputs) {
    inputs.velocityDPS = Units.rotationsToDegrees(motor.getEncoder().getVelocity());
    inputs.voltage = motor.getAppliedOutput() * motor.getBusVoltage();
    inputs.amps = motor.getOutputCurrent();
    inputs.commandedAngleDegs = this.targetRPM;
    inputs.angleDegrees = Units.rotationsToDegrees(motor.getEncoder().getPosition());
  }

  @Override
  public void setPID(LoggedTunableGains pid) {
    motor.configureAsync(
        ClimberConstants.createSparkConfig(),
        ResetMode.kNoResetSafeParameters,
        PersistMode.kNoPersistParameters);
  }

  @Override
  public void setServoPos(double pos) {
    servo.set(pos);
  }

  @Override
  public void setTargetAngle(double angle) {
    this.targetRPM = angle;
    motor
        .getClosedLoopController()
        .setReference(
            Units.degreesToRotations(angle), ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }
}
