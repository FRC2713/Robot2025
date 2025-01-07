package frc.robot.subsystems;

import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.util.Units;

public class KitbotMotorSparks implements KitbotmotorIO {
    private SparkMax motor;
    public KitbotMotorSparks() {
        this.motor = 
        new SparkMax (100,MotorType.kBrushless);
    }
    @Override

public void setVoltage(double Volts ){
    motor.setVoltage(Volts);

}
@Override
public void updateInputs(KitbotmotorInputs inputs){
    inputs.velocityRPM =  motor.getEncoder().getVelocity();
    inputs.MotorVoltage = motor.getAppliedOutput();
    inputs.IsOn = 
        Math.abs(Units.rotationsPerMinuteToRadiansPerSecond(motor.getEncoder().getVelocity()))
    > 0.005;
    inputs.position = motor.getEncoder().getPosition() * Math.PI * 2;

    

}
}
