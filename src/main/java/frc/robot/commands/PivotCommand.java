package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Pivot;

public class PivotCommand extends Command {
    public double DriverInput;
    public Pivot PivotThing;

    public PivotCommand (double DriverInput,Pivot PivotThing  ) {
        this.DriverInput = DriverInput;
        this.PivotThing = PivotThing;
    }

    public void initialize () {
        this.PivotThing.setTargetAngle(DriverInput);

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