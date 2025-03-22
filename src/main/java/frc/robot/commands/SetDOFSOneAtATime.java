package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.SSConstants;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

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
      BooleanSupplier intakingCoral,
      DoubleSupplier coralSpeed,
      DoubleSupplier algaeSpeed,
      DoubleSupplier elevatorTarget,
      DoubleSupplier shoulderTarget,
      DoubleSupplier wristTarget) {
    this.addCommands(
        RollerCmds.setEnableLimitSwitch(intakingCoral),
        RollerCmds.setSpeedAndWait(coralSpeed),
        AlgaeClawCmds.setSpeedAndWait(algaeSpeed),
        ElevatorCmds.setHeightAndWait(elevatorTarget),
        ShoulderCmds.setAngleAndWait(shoulderTarget),
        PivotCmds.setAngleAndWait(wristTarget));
  }

  /***
   * Convinence constructor for going to a scoring location. Moves Elevator, then Shoulder, then Pivot. Endeffector motors set instantly
   * @param elevatorTarget
   * @param shoulderTarget
   * @param wristTarget
   */
  public SetDOFSOneAtATime(
      DoubleSupplier elevatorTarget, DoubleSupplier shoulderTarget, DoubleSupplier wristTarget) {
    this(
        () -> false,
        () -> 0,
        SSConstants.AlgaeClaw.ALGAE_GRAB_SPEED,
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
      DoubleSupplier algaeSpeed,
      DoubleSupplier elevatorTarget,
      DoubleSupplier shoulderTarget,
      DoubleSupplier wristTarget) {
    this(() -> false, () -> 0, algaeSpeed, elevatorTarget, shoulderTarget, wristTarget);
  }
}
