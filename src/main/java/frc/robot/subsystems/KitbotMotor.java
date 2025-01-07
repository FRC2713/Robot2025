package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class KitbotMotor extends SubsystemBase{
    private final KitbotmotorInputsAutoLogged inputs;
    private final KitbotmotorIO IO;
    public KitbotMotor(KitbotmotorIO IO){
    this.inputs = new KitbotmotorInputsAutoLogged();
    IO.updateInputs(inputs);
    this.IO = IO;
    
    
    }
    
public void periodic() {
    IO.updateInputs(inputs);
    Logger.processInputs("KitbotMotor", inputs);
    


    
}

}