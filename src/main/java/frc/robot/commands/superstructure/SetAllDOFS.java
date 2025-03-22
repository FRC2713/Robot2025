package frc.robot.commands.superstructure;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.SetpointConstants;
import frc.robot.commands.AlgaeClawCmds;
import frc.robot.commands.ElevatorCmds;
import frc.robot.commands.PivotCmds;
import frc.robot.commands.RollerCmds;
import frc.robot.commands.ShoulderCmds;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

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
      String name,
      BooleanSupplier intakingCoral,
      DoubleSupplier coralSpeed,
      DoubleSupplier algaeSpeed,
      DoubleSupplier elevatorTarget,
      DoubleSupplier shoulderTarget,
      DoubleSupplier wristTarget) {
    this.addCommands(
        new InstantCommand(() -> Logger.recordOutput("Active SS", name)),
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
      String name,
      DoubleSupplier elevatorTarget,
      DoubleSupplier shoulderTarget,
      DoubleSupplier wristTarget) {
    this(
        name,
        () -> false,
        () -> 0,
        SetpointConstants.AlgaeClaw.ALGAE_GRAB_SPEED,
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
      String name,
      DoubleSupplier algaeSpeed,
      DoubleSupplier elevatorTarget,
      DoubleSupplier shoulderTarget,
      DoubleSupplier wristTarget) {
    this(name, () -> false, () -> 0, algaeSpeed, elevatorTarget, shoulderTarget, wristTarget);
  }
}
