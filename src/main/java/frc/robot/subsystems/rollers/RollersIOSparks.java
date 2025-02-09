package frc.robot.subsystems.rollers;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.constants.RollerConstants;

/** For the first implementation, the robot controls Tube and Algae with a single NEO */
public class RollersIOSparks implements RollersIO {

  private final SparkMax motor;
  private final SparkLimitSwitch limitSwitch;

  private double targetRPM;

  public RollersIOSparks() {
    this.motor = new SparkMax(RollerConstants.kCoralCANId, MotorType.kBrushless);
    this.limitSwitch = motor.getForwardLimitSwitch();
    motor.configure(
        RollerConstants.createCoralConfig(true),
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
    inputs.hasAlgae = motor.getOutputCurrent() > RollerConstants.kAlgaeCurrentThreshold;
  }

  @Override
  public void setTubeRPM(double rpm) {
    this.targetRPM = rpm;
    motor.set(rpm / RollerConstants.kAlgaeMaxVelocity);
  }

  @Override
  public void setEnableLimitSwitch(boolean setEnable) {
    motor.configure(
        RollerConstants.createCoralConfig(setEnable),
        ResetMode.kResetSafeParameters,
        PersistMode.kNoPersistParameters);
  }

  private boolean hasCoral() {
    return limitSwitch.isPressed();
  }
}
