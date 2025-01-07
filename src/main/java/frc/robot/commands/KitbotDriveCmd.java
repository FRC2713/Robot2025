package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.KitbotMotor;

public class KitbotDriveCmd extends Command {
    public double DriverInputNorm;
    public KitbotMotor Kitbot;

    public KitbotDriveCmd (double DriverInputNorm, KitbotMotor Kitbot ) {
        this.DriverInputNorm = DriverInputNorm;
        this.Kitbot = Kitbot;
    }
    public void initialize () {
       this.Kitbot.setCommandedVoltage(DriverInputNorm * 12);
    }
    public void execute () {
        System.out.println("Is executing");
    }
        public boolean isFinished () {
            return true;

    }
   
    public void end () {
        System.out.println("Is ended");
    }
}
