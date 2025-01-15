package frc.robot.subsystems.elevator;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import java.util.ResourceBundle.Control;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class ElevatorIOSparks implements ElevatorIO {

    private SparkMax left, right;

    public ElevatorIOSparks() {
        // Note: 100 is arbitrary and needs to be changed.
        left = new SparkMax(100, MotorType.kBrushless);
        right = new SparkMax(100, MotorType.kBrushless);

    }

    public void setVoltage(double volts1, double volts2) {
        left.setVoltage(volts1);
        right.setVoltage(volts2);
    }
    
    public void setTargetHeight(double height) {
        left.getClosedLoopController().setReference(height, ControlType.kMAXMotionPositionControl);
        right.getClosedLoopController().setReference(height, ControlType.kMAXMotionPositionControl);
    }
}