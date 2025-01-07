package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLog;

public interface KitbotmotorIO {
    @AutoLog
    public static class KitbotmotorInputs {
        public boolean IsOn = false;
        public double velocityRPM = 0.0;
        public double MotorVoltage = 0.0;
        public double position = 0.0;

    }
    

public void updateInputs(KitbotmotorInputs inputs);
public void setVoltage(double Volts );
}