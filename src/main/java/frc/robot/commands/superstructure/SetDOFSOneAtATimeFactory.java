package frc.robot.commands.superstructure;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.AlgaeClawCmds;
import frc.robot.commands.ElevatorCmds;
import frc.robot.commands.PivotCmds;
import frc.robot.commands.RollerCmds;
import frc.robot.commands.ShoulderCmds;

public class SetDOFSOneAtATimeFactory {
    List<Command> ordered = new ArrayList<>();
    boolean hasNames = false;


    public SetDOFSOneAtATimeFactory() {
        this.ordered = new ArrayList<>();
    }

    public SetDOFSOneAtATimeFactory addElevatorCommand(DoubleSupplier setpoint) {
        this.ordered.add(ElevatorCmds.setHeightAndWait(setpoint));
        return this;
    }

    public SetDOFSOneAtATimeFactory addShouldercommand(DoubleSupplier setpoint) {
        this.ordered.add(ShoulderCmds.setAngleAndWait(setpoint));
        return this;
    }

    public SetDOFSOneAtATimeFactory addPivotCommand(DoubleSupplier setpoint) {
        this.ordered.add(PivotCmds.setAngleAndWait(setpoint));
        return this;
    }

    public SetDOFSOneAtATimeFactory addCoralModeCommand(BooleanSupplier setpoint) {
        this.ordered.add(RollerCmds.setEnableLimitSwitch(setpoint));
        return this;
    }

    public SetDOFSOneAtATimeFactory addCoralSpeedCommand(DoubleSupplier setpoint) {
        this.ordered.add(RollerCmds.setSpeed(setpoint));
        return this;
    }

    public SetDOFSOneAtATimeFactory addAlgaeSpeedCommand(DoubleSupplier setpoint) {
        this.ordered.add(AlgaeClawCmds.setSpeed(setpoint));
        return this;
    }

    public SetDOFSOneAtATimeFactory addNames(String ssName, String eeName) {
        this.hasNames = true;
        this.ordered.add(0, new InstantCommand(() -> Logger.recordOutput("Active EE", eeName)));
        this.ordered.add(0, new InstantCommand(() -> Logger.recordOutput("Active SS", ssName)));
        return this;
    }

    public SetDOFSOneAtATime create() {
        if (!this.hasNames) {
            this.ordered.add(0, new InstantCommand(() -> Logger.recordOutput("Active EE", "UNKNOWN FACTORY COMMAND")));
            this.ordered.add(0, new InstantCommand(() -> Logger.recordOutput("Active SS", "UNKNOWN FACTORY COMMAND")));
        }
        
        return new SetDOFSOneAtATime(this.ordered);
    }


}