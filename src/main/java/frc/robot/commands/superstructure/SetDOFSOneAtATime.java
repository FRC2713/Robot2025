package frc.robot.commands.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.SetpointConstants;
import frc.robot.commands.AlgaeClawCmds;
import frc.robot.commands.ElevatorCmds;
import frc.robot.commands.PivotCmds;
import frc.robot.commands.RollerCmds;
import frc.robot.commands.ShoulderCmds;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class SetDOFSOneAtATime extends SequentialCommandGroup {

  /***
   * Moves Elevator, then Shoulder, then Pivot. Endeffector motors set instantly
   * @param intakingCoral
   * @param coralSpeed
   * @param algaeSpeed
   * @param elevatorTarget
   * @param shoulderTarget
   * @param wristTarget
   */
  public SetDOFSOneAtATime(
      String ssName,
      String eeName,
      BooleanSupplier intakingCoral,
      DoubleSupplier coralSpeed,
      DoubleSupplier algaeSpeed,
      DoubleSupplier elevatorTarget,
      DoubleSupplier shoulderTarget,
      DoubleSupplier wristTarget) {
    this.addCommands(
        new InstantCommand(() -> Logger.recordOutput("Active SS", ssName)),
        new InstantCommand(() -> Logger.recordOutput("Active EE", eeName)),
        RollerCmds.setEnableLimitSwitch(intakingCoral),
        RollerCmds.setSpeedAndWait(coralSpeed),
        AlgaeClawCmds.setSpeedAndWait(algaeSpeed),
        ElevatorCmds.setHeightAndWait(elevatorTarget),
        ShoulderCmds.setAngleAndWait(shoulderTarget),
        PivotCmds.setAngleAndWait(wristTarget));
  }

  /***
   * Meant to be for moving superstructures one at a time, built with factory
   * @param orderedCommands
   */
  public SetDOFSOneAtATime(List<Command> orderedCommands) {
    for (Command c : orderedCommands) {
      this.addCommands(c);
    }
  }

  /***
   * Convinence constructor for going to a scoring location. Moves Elevator, then Shoulder, then Pivot. Endeffector motors set instantly
   * @param elevatorTarget
   * @param shoulderTarget
   * @param wristTarget
   */
  public SetDOFSOneAtATime(
      String ssName,
      String eeName,
      DoubleSupplier elevatorTarget,
      DoubleSupplier shoulderTarget,
      DoubleSupplier wristTarget) {
    this(
        ssName,
        eeName,
        () -> false,
        () -> 0,
        SetpointConstants.AlgaeClaw.ALGAE_GRAB_SPEED,
        elevatorTarget,
        shoulderTarget,
        wristTarget);
  }

  /***
   * Convinence constructor for going to a scoring location. Moves Elevator, then Shoulder, then Pivot. Endeffector motors set instantly
   * @param algaeSpeed
   * @param elevatorTarget
   * @param shoulderTarget
   * @param wristTarget
   */
  public SetDOFSOneAtATime(
      String ssName,
      String eeName,
      DoubleSupplier algaeSpeed,
      DoubleSupplier elevatorTarget,
      DoubleSupplier shoulderTarget,
      DoubleSupplier wristTarget) {
    this(
        ssName,
        eeName,
        () -> false,
        () -> 0,
        algaeSpeed,
        elevatorTarget,
        shoulderTarget,
        wristTarget);
  }
}
