package frc.robot.subsystems.rollers;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.constants.RollerConstants;

/** For the first implementation, the robot controls Tube and Algae with a single NEO */
public class RollersIOSparks implements RollersIO {

  private final SparkFlex motor;
  private final SparkLimitSwitch limitSwitch;
  private double targetRPM;

  public RollersIOSparks() {
    this.motor = new SparkFlex(RollerConstants.kCANId, MotorType.kBrushless);
    this.limitSwitch = motor.getForwardLimitSwitch();
    motor.configure(
        RollerConstants.createConfig(60, true),
        ResetMode.kResetSafeParameters,
        PersistMode.kNoPersistParameters);
  }

  @Override
  public void updateInputs(RollersInputs inputs) {
    inputs.tubeVelocityRPM = motor.getEncoder().getVelocity();
    inputs.tubeOutputVoltage = motor.getAppliedOutput() * motor.getBusVoltage();
    inputs.tubeCurrentAmps = motor.getOutputCurrent();
    inputs.commandedTubeRPM = this.targetRPM;
    inputs.tubePositionDegs = Units.rotationsToDegrees(motor.getEncoder().getPosition());

    inputs.hasCoral = this.hasCoral();
  }

  @Override
  public void setRPM(double rpm) {
    this.targetRPM = rpm;
    motor.set(rpm / RollerConstants.kMaxVelocity);
  }

  @Override
  public void setEnableLimitSwitch(boolean setEnable) {
    motor.configureAsync(
        RollerConstants.createConfig(60, setEnable),
        ResetMode.kResetSafeParameters,
        PersistMode.kNoPersistParameters);
  }

  private boolean hasCoral() {
    return limitSwitch.isPressed();
  }

  @Override
  public boolean isAtTarget() {
    return Math.abs(targetRPM - motor.getEncoder().getVelocity())
        < RollerConstants.AT_TARGET_GIVE_RPM;
  }
}
