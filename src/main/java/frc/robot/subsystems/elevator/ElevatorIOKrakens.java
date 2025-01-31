package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.constants.ElevatorConstants;
import frc.robot.util.PhoenixUtil;

public class ElevatorIOKrakens implements ElevatorIO {
    private final TalonFX left;
    private final TalonFX right;

    private final PositionVoltage positionVoltageRequest = new PositionVoltage(0.0);
    private final VelocityVoltage velocityVoltageRequest = new VelocityVoltage(0.0);

    private final PositionTorqueCurrentFOC positionRequest = new PositionTorqueCurrentFOC(0.0);

    private TalonFXConfiguration configLeft;
    private TalonFXConfiguration configRight;

    public double lastHeight = ElevatorConstants.kInitialHeight;

    public ElevatorIOKrakens(TalonFXConfiguration configLeft, TalonFXConfiguration configRight) {
        this.configLeft = configLeft;
        this.configRight = configRight;
        left = new TalonFX(ElevatorConstants.kLeftCANId);
        right = new TalonFX(ElevatorConstants.kLeftCANId);
        
        var leftConfig = configLeft;
        leftConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        leftConfig.Feedback.SensorToMechanismRatio = ElevatorConstants.kGearReduction;
        leftConfig.TorqueCurrent.PeakForwardTorqueCurrent = ElevatorConstants.kMaxCurrentLimit;
        leftConfig.TorqueCurrent.PeakReverseTorqueCurrent = -ElevatorConstants.kMaxCurrentLimit;
        //change this
        leftConfig.CurrentLimits.StatorCurrentLimit = ElevatorConstants.kMaxCurrentLimit;
        leftConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        leftConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        PhoenixUtil.tryUntilOk(5, () -> left.getConfigurator().apply(leftConfig, 0.25));
        PhoenixUtil.tryUntilOk(5, () -> left.setPosition(ElevatorConstants.kInitialHeight, 0.25));

        var rightConfig = configRight;
        rightConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        rightConfig.Feedback.SensorToMechanismRatio = ElevatorConstants.kGearReduction;
        rightConfig.TorqueCurrent.PeakForwardTorqueCurrent = ElevatorConstants.kMaxCurrentLimit;
        rightConfig.TorqueCurrent.PeakReverseTorqueCurrent = -ElevatorConstants.kMaxCurrentLimit;
        //change this
        rightConfig.CurrentLimits.StatorCurrentLimit = ElevatorConstants.kMaxCurrentLimit;
        rightConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        rightConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        PhoenixUtil.tryUntilOk(5, () -> right.getConfigurator().apply(rightConfig, 0.25));
        PhoenixUtil.tryUntilOk(5, () -> right.setPosition(ElevatorConstants.kInitialHeight, 0.25));

    }
    private double getAvgPosition() {
        return (left.getPosition().getValueAsDouble() + right.getPosition().getValueAsDouble()) / 2.0;
    }
    public void setVoltage(double volts1, double volts2) {
        left.setVoltage(volts1);
        right.setVoltage(volts2);
    }
    public void setTargetHeight(double height) {
        left.setControl(positionRequest.withPosition(height));
        right.setControl(positionRequest.withPosition(height));
        lastHeight = height;
    }
    public boolean isAtTarget() {
        return Math.abs(Units.metersToInches(getAvgPosition()) - lastHeight) <= 1;
    }
}
