package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.SSConstants;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class SetAllDOFS extends ParallelCommandGroup {

  /***
   * Set all DOFs of the superstructure and speeds for the end effector rollers
   * @param intakingCoral
   * @param coralSpeed
   * @param algaeSpeed
   * @param elevatorTarget
   * @param shoulderTarget
   * @param wristTarget
   */
  public SetAllDOFS(
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
        PivotCmds.setAngleAndWait(wristTarget),
        ShoulderCmds.setAngleAndWait(shoulderTarget));
  }

  /***
   * Convinence constructor for going to a scoring location
   * @param elevatorTarget
   * @param shoulderTarget
   * @param wristTarget
   */
  public SetAllDOFS(
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
   * Convinence constructor for going to a scoring location
   * @param algaeSpeed
   * @param elevatorTarget
   * @param shoulderTarget
   * @param wristTarget
   */
  public SetAllDOFS(
      DoubleSupplier algaeSpeed,
      DoubleSupplier elevatorTarget,
      DoubleSupplier shoulderTarget,
      DoubleSupplier wristTarget) {
    this(() -> false, () -> 0, algaeSpeed, elevatorTarget, shoulderTarget, wristTarget);
  }
}
