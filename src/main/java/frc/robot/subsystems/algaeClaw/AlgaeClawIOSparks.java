package frc.robot.subsystems.algaeClaw;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.util.Units;
import frc.robot.SetpointConstants;
import frc.robot.subsystems.constants.AlgaeClawConstants;
import frc.robot.subsystems.constants.RollerConstants;

/** For the first implementation, the robot controls AlgaeClaw and Algae with a single NEO */
public class AlgaeClawIOSparks implements AlgaeClawIO {

  private final SparkFlex motor;
  private double targetRPM;
  private Debouncer debouncer;
  private boolean enableLS = true;

  public AlgaeClawIOSparks() {
    this.motor = new SparkFlex(AlgaeClawConstants.kCANId, MotorType.kBrushless);
    motor.configure(
        AlgaeClawConstants.createConfig(80),
        ResetMode.kResetSafeParameters,
        PersistMode.kNoPersistParameters);
    debouncer = new Debouncer(0.25, DebounceType.kBoth);
  }

  @Override
  public void updateInputs(AlgaeClawInputs inputs) {
    if (enableLS && hasAlgae()) {
      setRPM(SetpointConstants.AlgaeClaw.ALGAE_HOLD_SPEED.getAsDouble());
    }
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
    motor.set(rpm / AlgaeClawConstants.kMaxVelocity);
  }

  @Override
  public void setEnableLimitSwitch(boolean setEnable) {
    enableLS = setEnable;
  }

  private boolean hasAlgae() {
    return debouncer.calculate(
        motor.getOutputCurrent()
            > SetpointConstants.AlgaeClaw.ALGAE_DETECTED_CURRENT_LIMIT.getAsDouble());
  }

  @Override
  public boolean isAtTarget() {
    return Math.abs(targetRPM - motor.getEncoder().getVelocity())
        < RollerConstants.AT_TARGET_GIVE_RPM;
  }
}
