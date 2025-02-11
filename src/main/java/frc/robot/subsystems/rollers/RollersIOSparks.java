package frc.robot.subsystems.rollers;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.constants.RollerConstants;

/** For the first implementation, the robot controls Tube and Algae with a single NEO */
public class RollersIOSparks implements RollersIO {

  private final SparkFlex motor;
  private final SparkLimitSwitch limitSwitch;

  private double targetRPM;

  private final SparkFlexConfig config = RollerConstants.createCoralConfig();

  public RollersIOSparks() {
    this.motor = new SparkFlex(RollerConstants.kCoralCANId, MotorType.kBrushless);
    this.limitSwitch = motor.getForwardLimitSwitch();
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
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
    if (setEnable) {
      config.limitSwitch.apply(
          (new LimitSwitchConfig())
              .forwardLimitSwitchEnabled(true)
              .forwardLimitSwitchType(Type.kNormallyOpen));
    } else {
      config.limitSwitch.apply((new LimitSwitchConfig()).forwardLimitSwitchEnabled(false));
    }
  }

  private boolean hasCoral() {
    return limitSwitch.isPressed();
  }
}
