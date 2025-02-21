package frc.robot.subsystems.algaeClaw;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.constants.RollerConstants;

/** For the first implementation, the robot controls AlgaeClaw and Algae with a single NEO */
public class AlgaeClawIOSparks implements AlgaeClawIO {

  private final SparkFlex motor;
  private final SparkLimitSwitch limitSwitch;
  private double targetRPM;

  public boolean enableLS = true;

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
    inputs.algaeClawVelocityRPM = motor.getEncoder().getVelocity();
    inputs.algaeClawOutputVoltage = motor.getAppliedOutput() * motor.getBusVoltage();
    inputs.algaeClawCurrentAmps = motor.getOutputCurrent();
    inputs.commandedAlgaeClawRPM = this.targetRPM;
    inputs.algaeClawPositionDegs = Units.rotationsToDegrees(motor.getEncoder().getPosition());

    inputs.hasAlgae = this.hasAlgae();
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

  private boolean hasAlgae() {
    return limitSwitch.isPressed();
  }
}
