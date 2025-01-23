package frc.robot.subsystems.elevator;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.util.Units;

public class ElevatorIOSparks implements ElevatorIO {
  private SparkMax left, right;
  SparkMaxConfig leftConfig = new SparkMaxConfig();
  SparkMaxConfig rightConfig = new SparkMaxConfig();

  public double lastHeight = 0.0;

  public ElevatorIOSparks() {
    // TODO: 100 is arbitrary and needs to be changed.
    left = new SparkMax(100, MotorType.kBrushless);
    right = new SparkMax(100, MotorType.kBrushless);
    leftConfig.smartCurrentLimit(0).secondaryCurrentLimit(0);
    leftConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    left.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    rightConfig.smartCurrentLimit(0).secondaryCurrentLimit(0);
    rightConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    right.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
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
  }

  @Override
  public void updateInputs(ElevatorInputs inputs) {
    throw new UnsupportedOperationException("Unimplemented method 'updateInputs'");
  }

  @Override
  public void reset() {
    throw new UnsupportedOperationException("Unimplemented method 'reset'");
  }

  @Override
  public boolean shouldApplyFF() {
    throw new UnsupportedOperationException("Unimplemented method 'shouldApplyFF'");
  }

  @Override
  public void setCurrentLimits() {
    throw new UnsupportedOperationException("Unimplemented method 'setCurrentLimits'");
  }

  public boolean isAtTarget() {
    return Math.abs(Units.metersToInches(getAvgPosition()) - lastHeight) <= 1;
  }
}
