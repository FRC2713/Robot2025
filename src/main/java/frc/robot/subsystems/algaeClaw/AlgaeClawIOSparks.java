package frc.robot.subsystems.algaeClaw;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.constants.RollerConstants;

/** For the first implementation, the robot controls Tube and Algae with a single NEO */
public class AlgaeClawIOSparks implements AlgaeClawIO {

  private final SparkFlex motor;
  private final SparkLimitSwitch limitSwitch;
  private double targetRPM;

  public boolean enableLS = true;
  public boolean enableAlgaeLS = false;

  public AlgaeClawIOSparks() {
    this.motor = new SparkFlex(RollerConstants.kCoralCANId, MotorType.kBrushless);
    this.limitSwitch = motor.getReverseLimitSwitch();
    motor.configure(
        RollerConstants.createCoralConfig(60),
        ResetMode.kResetSafeParameters,
        PersistMode.kNoPersistParameters);
  }

  @Override
  public void updateInputs(AlgaeClawInputs inputs) {
    if (this.hasCoral() && enableLS) {
      setTubeRPM(0);
    }

    inputs.tubeVelocityRPM = motor.getEncoder().getVelocity();
    inputs.tubeOutputVoltage = motor.getAppliedOutput() * motor.getBusVoltage();
    inputs.tubeCurrentAmps = motor.getOutputCurrent();
    inputs.commandedTubeRPM = this.targetRPM;
    inputs.tubePositionDegs = Units.rotationsToDegrees(motor.getEncoder().getPosition());

    inputs.hasCoral = this.hasCoral();
    inputs.hasAlgae = motor.getOutputCurrent() > RollerConstants.kAlgaeCurrentThreshold;
    if (inputs.hasAlgae && enableAlgaeLS) {
      setTubeRPM(-500);
    }
  }

  @Override
  public void setTubeRPM(double rpm) {
    this.targetRPM = rpm;
    motor.set(rpm / RollerConstants.kAlgaeMaxVelocity);
  }

  @Override
  public void setEnableLimitSwitch(boolean setEnable) {
    enableLS = setEnable;
  }

  @Override
  public void setEnableAlgaeLimitSwitch(boolean setEnable) {
    enableAlgaeLS = setEnable;
  }

  private boolean hasCoral() {
    return limitSwitch.isPressed();
  }
}
