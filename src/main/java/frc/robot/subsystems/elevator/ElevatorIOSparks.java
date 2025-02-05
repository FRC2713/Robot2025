package frc.robot.subsystems.elevator;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.constants.ElevatorConstants;

public class ElevatorIOSparks implements ElevatorIO {
  private SparkMax left, right;
  SparkMaxConfig leftConfig = new SparkMaxConfig();
  SparkMaxConfig rightConfig = new SparkMaxConfig();

  public double lastHeight = 0.0;

  public ElevatorIOSparks() {
    left = new SparkMax(ElevatorConstants.kLeftCANId, MotorType.kBrushless);
    right = new SparkMax(ElevatorConstants.kRightCANId, MotorType.kBrushless);

    left.configure(
        ElevatorConstants.createLeftSparkMaxConfig(),
        ResetMode.kResetSafeParameters,
        PersistMode.kNoPersistParameters);
    right.configure(
        ElevatorConstants.createRightSparkMaxConfig(),
        ResetMode.kResetSafeParameters,
        PersistMode.kNoPersistParameters);
  }

  private double getAvgPosition() {
    return (left.getEncoder().getPosition() + right.getEncoder().getPosition()) / 2;
  }

  public void setVoltage(double volts1, double volts2) {
    left.setVoltage(volts1);
    right.setVoltage(volts2);
  }

  public void setTargetHeight(double height) {
    left.getClosedLoopController().setReference(height, ControlType.kMAXMotionPositionControl);
    right.getClosedLoopController().setReference(height, ControlType.kMAXMotionPositionControl);
    lastHeight = height;
  }

  @Override
  public void updateInputs(ElevatorInputs inputs) {
    inputs.outputVoltageRight = right.getAppliedOutput();
    inputs.heightInchesRight = right.getEncoder().getPosition();
    inputs.velocityInchesPerSecondRight = right.getEncoder().getVelocity() / 60;
    inputs.tempCelsiusRight = 0.0;
    inputs.currentDrawAmpsRight = right.getOutputCurrent();

    inputs.outputVoltageLeft = left.getAppliedOutput();
    inputs.heightInchesLeft = left.getEncoder().getPosition();
    inputs.velocityInchesPerSecondLeft = left.getEncoder().getVelocity() / 60;
    inputs.tempCelsiusLeft = 0.0;
    inputs.currentDrawAmpsLeft = left.getOutputCurrent();
  }

  public boolean isAtTarget() {
    return Math.abs(Units.metersToInches(getAvgPosition()) - lastHeight) <= 1;
  }
}
