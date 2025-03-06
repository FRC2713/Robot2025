package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.subsystems.constants.ElevatorConstants;
import frc.robot.util.LoggedTunableGains;
import frc.robot.util.PhoenixUtil;

public class ElevatorIOKrakens implements ElevatorIO {
  private final TalonFX left;
  private final TalonFX right;
  private final TalonFXConfiguration leftConfig;
  private final TalonFXConfiguration rightConfig;

  private final MotionMagicExpoTorqueCurrentFOC heightRequest =
      new MotionMagicExpoTorqueCurrentFOC(0);

  public double lastHeight = ElevatorConstants.kInitialHeight;

  public ElevatorIOKrakens() {
    left = new TalonFX(ElevatorConstants.kLeftCANId);
    right = new TalonFX(ElevatorConstants.kRightCANId);

    leftConfig = createKrakenConfig(false);
    PhoenixUtil.tryUntilOk(5, () -> left.getConfigurator().apply(leftConfig, 0.25));
    PhoenixUtil.tryUntilOk(
        5,
        () ->
            left.setPosition(
                ElevatorConstants.kInitialHeight * ElevatorConstants.kRotationsToHeightConversion,
                0.25));

    rightConfig = createKrakenConfig(true);
    PhoenixUtil.tryUntilOk(5, () -> right.getConfigurator().apply(rightConfig, 0.25));
    PhoenixUtil.tryUntilOk(
        5,
        () ->
            right.setPosition(
                ElevatorConstants.kInitialHeight * ElevatorConstants.kRotationsToHeightConversion,
                0.25));
  }

  public TalonFXConfiguration createKrakenConfig(boolean inverted) {
    var config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Feedback.SensorToMechanismRatio = ElevatorConstants.kGearReduction;
    config.TorqueCurrent.PeakForwardTorqueCurrent = ElevatorConstants.kMaxCurrentLimit;
    config.TorqueCurrent.PeakReverseTorqueCurrent = -ElevatorConstants.kMaxCurrentLimit;
    config.CurrentLimits.StatorCurrentLimit = ElevatorConstants.kMaxCurrentLimit;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.MotorOutput.Inverted =
        (inverted) ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive;

    // set slot 0 gains
    config.Slot0 = ElevatorConstants.Gains.toTalonFX();

    // set Motion Magic settings
    config.MotionMagic = ElevatorConstants.Gains.getMotionMagicConfig();

    config.ClosedLoopGeneral.ContinuousWrap = false;
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        ElevatorConstants.toRotations(ElevatorConstants.kMaxHeight);
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        ElevatorConstants.toRotations(ElevatorConstants.kMinHeight);

    return config;
  }

  @Override
  public void setPID(LoggedTunableGains slot0pid) {
    System.out.println("Updating elevator PID");
    leftConfig.Slot0 = slot0pid.toTalonFX();
    rightConfig.Slot0 = slot0pid.toTalonFX();

    leftConfig.MotionMagic = slot0pid.getMotionMagicConfig();
    rightConfig.MotionMagic = slot0pid.getMotionMagicConfig();

    PhoenixUtil.tryUntilOk(5, () -> left.getConfigurator().apply(leftConfig, 0.25));
    PhoenixUtil.tryUntilOk(5, () -> right.getConfigurator().apply(rightConfig, 0.25));
  }

  private double getLeftHeight() {
    return left.getPosition().getValueAsDouble() * ElevatorConstants.kRotationsToHeightConversion;
  }

  private double getRightHeight() {
    return right.getPosition().getValueAsDouble() * ElevatorConstants.kRotationsToHeightConversion;
  }

  private double getAvgHeight() {
    return (getLeftHeight() + getRightHeight()) / 2.0;
  }

  @Override
  public void setVoltage(double volts1, double volts2) {
    left.setVoltage(volts1);
    right.setVoltage(volts2);
  }

  @Override
  public void setTargetHeight(double height) {
    left.setControl(
        heightRequest.withPosition(height / ElevatorConstants.kRotationsToHeightConversion));
    right.setControl(
        heightRequest.withPosition(height / ElevatorConstants.kRotationsToHeightConversion));
    lastHeight = height;
  }

  @Override
  public boolean isAtTarget() {
    return Math.abs(getAvgHeight() - lastHeight) <= ElevatorConstants.AT_TARGET_GIVE_INCHES;
  }

  @Override
  public void updateInputs(ElevatorInputs inputs) {
    inputs.outputVoltageRight = right.getMotorVoltage().getValueAsDouble();
    inputs.heightInchesRight = getRightHeight();
    inputs.velocityInchesPerSecondRight =
        right.getVelocity().getValueAsDouble() * ElevatorConstants.kRotationsToHeightConversion;
    inputs.tempCelsiusRight = right.getDeviceTemp().getValueAsDouble();
    inputs.currentDrawAmpsRight = Math.abs(right.getTorqueCurrent().getValueAsDouble());

    inputs.outputVoltageLeft = left.getMotorVoltage().getValueAsDouble();
    inputs.heightInchesLeft = getLeftHeight();
    inputs.velocityInchesPerSecondLeft =
        left.getVelocity().getValueAsDouble() * ElevatorConstants.kRotationsToHeightConversion;
    inputs.tempCelsiusLeft = left.getDeviceTemp().getValueAsDouble();
    inputs.currentDrawAmpsLeft = Math.abs(left.getTorqueCurrent().getValueAsDouble());

    inputs.commandedHeightInches = lastHeight;
  }

  @Override
  public void changeSlot(int slot) {
    heightRequest.withSlot(slot);
    setTargetHeight(lastHeight);
  }
}
