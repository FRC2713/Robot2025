package frc.robot.commands.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.AlgaeClawCmds;
import frc.robot.commands.ElevatorCmds;
import frc.robot.commands.RollerCmds;
import frc.robot.commands.ShoulderCmds;
import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class SetDOFSOneAtATimeFactory {
  List<Command> ordered = new ArrayList<>();

  public SetDOFSOneAtATimeFactory(String ssName, String eeName) {
    this.ordered = new ArrayList<>();
    this.ordered.add(new InstantCommand(() -> Logger.recordOutput("Active SS", ssName)));
    this.ordered.add(new InstantCommand(() -> Logger.recordOutput("Active EE", eeName)));
  }

  public SetDOFSOneAtATimeFactory addElevatorCommand(DoubleSupplier setpoint) {
    this.ordered.add(ElevatorCmds.setHeightAndWait(setpoint));
    return this;
  }

  public SetDOFSOneAtATimeFactory addShoulderCommand(DoubleSupplier setpoint) {
    this.ordered.add(ShoulderCmds.setAngleAndWait(setpoint));
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

  public SetDOFSOneAtATimeFactory addWait(DoubleSupplier seconds) {
    this.ordered.add(Commands.waitSeconds(seconds.getAsDouble()));
    return this;
  }

  public SetDOFSOneAtATime create() {
    return new SetDOFSOneAtATime(this.ordered);
  }
}
