package frc.robot.subsystems.endEffector;

import com.revrobotics.spark.SparkAnalogSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.constants.AlgaeClawConstants;
import frc.robot.subsystems.constants.RollerConstants;

/** For the first implementation, the robot controls Tube and Algae with a single NEO */
public class EndEffectorIOSparks implements EndEffectorIO {

  private final SparkFlex coralMotor;
  private final SparkFlex algaeMotor;
  private final SparkLimitSwitch limitSwitch;
  private final SparkAnalogSensor algaeSensor;
  private final Debouncer hasAlgaeDebounce = new Debouncer(0.5, DebounceType.kBoth);
  private double targetCoralRPM;
  private double targetAlgaeRPM;

  public EndEffectorIOSparks() {
    this.coralMotor = new SparkFlex(RollerConstants.kCANId, MotorType.kBrushless);
    this.limitSwitch = coralMotor.getForwardLimitSwitch();
    this.algaeMotor = new SparkFlex(AlgaeClawConstants.kCANId, MotorType.kBrushless);
    this.algaeSensor = coralMotor.getAnalog();
    coralMotor.configure(
        RollerConstants.createConfig(60, true),
        ResetMode.kResetSafeParameters,
        PersistMode.kNoPersistParameters);

    algaeMotor.configure(
        AlgaeClawConstants.createConfig(50),
        ResetMode.kResetSafeParameters,
        PersistMode.kNoPersistParameters);
  }

  @Override
  public void updateInputs(EndEffectorInputs inputs) {
    inputs.tubeVelocityRPM = coralMotor.getEncoder().getVelocity();
    inputs.tubeOutputVoltage = coralMotor.getAppliedOutput() * coralMotor.getBusVoltage();
    inputs.tubeCurrentAmps = coralMotor.getOutputCurrent();
    inputs.commandedTubeRPM = this.targetCoralRPM;
    inputs.tubePositionDegs = Units.rotationsToDegrees(coralMotor.getEncoder().getPosition());
    inputs.algaeRollersVelocityRPM = algaeMotor.getEncoder().getVelocity();
    inputs.algaeRollersOutputVoltage = algaeMotor.getAppliedOutput() * algaeMotor.getBusVoltage();
    inputs.algaeRollersCurrentAmps = algaeMotor.getOutputCurrent();
    inputs.commandedAlgaeRollersRPM = this.targetAlgaeRPM;
    inputs.algaeRollersPositionDegs =
        Units.rotationsToDegrees(algaeMotor.getEncoder().getPosition());
    inputs.algaeSensorDistance = algaeSensor.getVoltage();
    inputs.hasAlgae = this.hasAlgae();
    inputs.hasCoral = this.hasCoral();
  }

  @Override
  public void setCoralRPM(double rpm) {
    this.targetCoralRPM = rpm;
    coralMotor.set(rpm / RollerConstants.kMaxVelocity);
  }

  @Override
  public void setAlgaeRPM(double rpm) {
    this.targetAlgaeRPM = rpm;
    algaeMotor.set(rpm / AlgaeClawConstants.kMaxVelocity);
  }

  @Override
  public void setAlgaeVoltage(double volts) {
    algaeMotor.getClosedLoopController().setReference(volts, ControlType.kVoltage);
  }

  @Override
  public void setEnableLimitSwitch(boolean setEnable) {
    coralMotor.configureAsync(
        RollerConstants.createConfig(60, setEnable),
        ResetMode.kResetSafeParameters,
        PersistMode.kNoPersistParameters);
  }

  private boolean hasCoral() {
    return limitSwitch.isPressed();
  }

  private boolean hasAlgae() {
    return hasAlgaeDebounce.calculate(
        algaeSensor.getVoltage() > AlgaeClawConstants.hasAlgaeVoltage);
  }

  @Override
  public boolean isCoralAtTarget() {
    return Math.abs(targetCoralRPM - coralMotor.getEncoder().getVelocity())
        < RollerConstants.AT_TARGET_GIVE_RPM;
  }

  @Override
  public boolean isAlgaeAtTarget() {
    return Math.abs(targetAlgaeRPM - algaeMotor.getEncoder().getVelocity())
        < AlgaeClawConstants.AT_TARGET_GIVE_RPM;
  }

  @Override
  public void setCoralCurrentLimit(int currentLimit) {
    coralMotor.configureAsync(
        RollerConstants.createConfig(currentLimit, true),
        ResetMode.kResetSafeParameters,
        PersistMode.kNoPersistParameters);
  }

  @Override
  public void setAlgaeCurrentLimit(int algaeCurrentLimit) {
    algaeMotor.configureAsync(
        AlgaeClawConstants.createConfig(algaeCurrentLimit),
        ResetMode.kResetSafeParameters,
        PersistMode.kNoPersistParameters);
  }
}
