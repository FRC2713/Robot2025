package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
// import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.constants.ElevatorConstants;
import frc.robot.util.PhoenixUtil;

public class ElevatorIOKrakens implements ElevatorIO {
  private final TalonFX left;
  private final TalonFX right;

  private final MotionMagicExpoTorqueCurrentFOC heightRequest =
      new MotionMagicExpoTorqueCurrentFOC(0);

  public double lastHeight = ElevatorConstants.kInitialHeight;

  public ElevatorIOKrakens() {
    left = new TalonFX(ElevatorConstants.kLeftCANId);
    right = new TalonFX(ElevatorConstants.kRightCANId);

    var leftConfig = ElevatorConstants.createKrakenConfig(false);
    PhoenixUtil.tryUntilOk(5, () -> left.getConfigurator().apply(leftConfig, 0.25));
    PhoenixUtil.tryUntilOk(
        5,
        () ->
            left.setPosition(
                ElevatorConstants.kInitialHeight * ElevatorConstants.kRotationsToHeightConversion,
                0.25));

    var rightConfig = ElevatorConstants.createKrakenConfig(true);
    PhoenixUtil.tryUntilOk(5, () -> right.getConfigurator().apply(rightConfig, 0.25));
    PhoenixUtil.tryUntilOk(
        5,
        () ->
            right.setPosition(
                ElevatorConstants.kInitialHeight * ElevatorConstants.kRotationsToHeightConversion,
                0.25));
  }

  private double getAvgPosition() {
    return (left.getPosition().getValueAsDouble() + right.getPosition().getValueAsDouble()) / 2.0;
  }

  public void setVoltage(double volts1, double volts2) {
    left.setVoltage(volts1);
    right.setVoltage(volts2);
  }

  public void setTargetHeight(double height) {
    left.setControl(
        heightRequest.withPosition(height / ElevatorConstants.kRotationsToHeightConversion));
    right.setControl(
        heightRequest.withPosition(height / ElevatorConstants.kRotationsToHeightConversion));
    lastHeight = height;
  }

  public boolean isAtTarget() {
    return Math.abs(Units.metersToInches(getAvgPosition()) - lastHeight) <= 1;
  }

  public void updateInputs(ElevatorInputs inputs) {
    inputs.outputVoltageRight = right.getMotorVoltage().getValueAsDouble();
    inputs.heightInchesRight =
        right.getPosition().getValueAsDouble() * ElevatorConstants.kRotationsToHeightConversion;
    inputs.velocityInchesPerSecondRight =
        right.getVelocity().getValueAsDouble() * ElevatorConstants.kRotationsToHeightConversion;
    inputs.tempCelsiusRight = 0.0;
    inputs.currentDrawAmpsRight = Math.abs(right.getTorqueCurrent().getValueAsDouble());

    inputs.outputVoltageLeft = left.getMotorVoltage().getValueAsDouble();
    inputs.heightInchesLeft =
        left.getPosition().getValueAsDouble() * ElevatorConstants.kRotationsToHeightConversion;
    inputs.velocityInchesPerSecondLeft =
        left.getVelocity().getValueAsDouble() * ElevatorConstants.kRotationsToHeightConversion;
    inputs.tempCelsiusLeft = 0.0;
    inputs.currentDrawAmpsLeft = Math.abs(left.getTorqueCurrent().getValueAsDouble());
    inputs.commandedHeightInches = lastHeight;
  }
}
