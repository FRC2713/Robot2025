package frc.robot.commands.superstructure;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.SetpointConstants;
import frc.robot.commands.ClimberCmds;
import frc.robot.commands.ElevatorCmds;
import frc.robot.commands.PivotCmds;
import frc.robot.commands.ShoulderCmds;

import org.littletonrobotics.junction.Logger;

public class SetClimbingConfig extends SequentialCommandGroup {

  public SetClimbingConfig(String name) {
    this.addCommands(
        new InstantCommand(() -> Logger.recordOutput("Active SS", name)),
        // move the arm out of the way, elevator first
        Commands.parallel(
            ElevatorCmds.setHeightAndWait(SetpointConstants.Elevator.CLIMB_PREP_HEIGHT),
            ShoulderCmds.setAngle(SetpointConstants.Shoulder.PREP_CLIMB_ANGLE_DEGS),
            PivotCmds.setAngle(SetpointConstants.Pivot.PREP_CLIMB_ANGLE_DEGS)),
        // deploy the climber
        ClimberCmds.deploy(),
        Commands.waitSeconds(0.5),
        // tuck the arm back in
        Commands.sequence(
            ShoulderCmds.setAngle(SetpointConstants.Shoulder.CLIMB_ANGLE_DEGS),
            PivotCmds.setAngle(SetpointConstants.Pivot.CLIMB_ANGLE_DEGS),
            ElevatorCmds.setHeight(0)));
  }
}
