package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

//Intake is essentially Shoulder and EndEfffector mashed into one class

public class Intake extends SubsystemBase {
    private final IntakeIO IO;

    public Intake(IntakeIO IO) {
        this.IO = IO;
    }
    
}
