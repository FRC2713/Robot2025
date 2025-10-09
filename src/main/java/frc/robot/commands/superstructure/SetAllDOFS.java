package frc.robot.commands.superstructure;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.AlgaeClawCmds;
import frc.robot.commands.ArmCmds;
import frc.robot.commands.ElevatorCmds;
import frc.robot.commands.RollerCmds;
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
   * @param armTarget
   */
  public SetAllDOFS(
      String ssName,
      String eeName,
      BooleanSupplier intakingCoral,
      DoubleSupplier coralSpeed,
      DoubleSupplier algaeSpeed,
      DoubleSupplier elevatorTarget,
      DoubleSupplier armTarget) {
    this.addCommands(
        new InstantCommand(() -> Logger.recordOutput("Active SS", ssName)),
        new InstantCommand(() -> Logger.recordOutput("Active EE", eeName)),
        RollerCmds.setEnableLimitSwitch(intakingCoral),
        RollerCmds.setSpeed(coralSpeed),
        AlgaeClawCmds.setSpeed(algaeSpeed),
        ElevatorCmds.setHeightAndWait(elevatorTarget),
        ArmCmds.armSetAngleAndWait(armTarget));
  }

  /***
   * Convinence constructor for going to a coral scoring location but not scoring yet
   * @param algaeSpeed
   * @param elevatorTarget
   * @param shoulderTarget
   */
  public SetAllDOFS(
      String ssName,
      String eeName,
      DoubleSupplier algaeSpeed,
      DoubleSupplier elevatorTarget,
      DoubleSupplier shoulderTarget) {
    this(ssName, eeName, () -> false, () -> 0, algaeSpeed, elevatorTarget, shoulderTarget);
  }
}
