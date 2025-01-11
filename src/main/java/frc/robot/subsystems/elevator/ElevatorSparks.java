package frc.robot.subsystems.elevator;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.subsystems.constants.ElevatorConstants;

public class ElevatorSparks implements ElevatorIO {

  private SparkMax left, right;
  private double targetHeight;

  public ElevatorSparks() {
    // Note: 100 is arbitrary and needs to be changed.
    left = new SparkMax(ElevatorConstants.kLeftCANId, MotorType.kBrushless);
    this.left.configure(
        ElevatorConstants.createLeftSparkMaxConfig(),
        ResetMode.kResetSafeParameters,
        PersistMode.kNoPersistParameters);

    right = new SparkMax(ElevatorConstants.kRightCANId, MotorType.kBrushless);
    this.right.configure(
        ElevatorConstants.createRightSparkMaxConfig(),
        ResetMode.kResetSafeParameters,
        PersistMode.kNoPersistParameters);
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
    inputs.currentDrawAmpsLeft = left.getOutputCurrent();
    inputs.outputVoltageLeft =
        MathUtil.clamp(left.getAppliedOutput() * RobotController.getBatteryVoltage(), -12.0, 12.0);
    inputs.heightInchesLeft = left.getEncoder().getPosition();
    inputs.velocityInchesPerSecondLeft = left.getEncoder().getVelocity();
    inputs.tempCelsiusLeft = left.getMotorTemperature();

    inputs.currentDrawAmpsRight = right.getOutputCurrent();
    inputs.outputVoltageRight =
        MathUtil.clamp(right.getAppliedOutput() * RobotController.getBatteryVoltage(), -12.0, 12.0);
    inputs.heightInchesRight = right.getEncoder().getPosition();
    inputs.velocityInchesPerSecondRight = right.getEncoder().getVelocity();
    inputs.tempCelsiusRight = right.getMotorTemperature();

    inputs.commandedHeight = targetHeight;
  }
}
