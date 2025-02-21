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

  public boolean enableLS = true;
  public boolean enableAlgaeLS = false;

  public RollersIOSparks() {
    this.motor = new SparkFlex(RollerConstants.kCoralCANId, MotorType.kBrushless);
    this.limitSwitch = motor.getReverseLimitSwitch();
    motor.configure(
        RollerConstants.createCoralConfig(60),
        ResetMode.kResetSafeParameters,
        PersistMode.kNoPersistParameters);
  }

  @Override
  public void updateInputs(RollersInputs inputs) {
    if (this.hasCoral() && enableLS) {
      setRPM(0);
    }

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
    motor.set(rpm / RollerConstants.kAlgaeMaxVelocity);
  }

  @Override
  public void setEnableLimitSwitch(boolean setEnable) {
    enableLS = setEnable;
  }

  private boolean hasCoral() {
    return limitSwitch.isPressed();
  }
}
