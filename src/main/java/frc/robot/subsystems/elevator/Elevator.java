package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.constants.ElevatorConstants;
import frc.robot.util.LoggedTunableGains;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private ElevatorFeedforward feedforward = ElevatorConstants.Gains.createElevatorFF();
  private PIDController pid = ElevatorConstants.Gains.createPIDController();
  private final DCMotor motor = DCMotor.getKrakenX60Foc(2);
  private final ElevatorSim sim =
      new ElevatorSim(
          motor,
          ElevatorConstants.kGearReduction,
          ElevatorConstants.kCarriageMass,
          ElevatorConstants.kDrumRadius,
          ElevatorConstants.kMinHeight,
          ElevatorConstants.kMaxHeight,
          true,
          ElevatorConstants.kInitialHeight);
  public double setpoint = 0.0;

  // 3d visualisation stuff, dw bout it
  public Pose3d pose = ElevatorConstants.kInitialPose;
  public Transform3d transform = new Transform3d();

  public Elevator() {}

  @Override
  public void periodic() {

    double pidOutput = pid.calculate(getCurrentHeight(), setpoint);
    double feedforwardOutput =
        feedforward.calculate(Units.metersToInches(sim.getVelocityMetersPerSecond()));

    Logger.recordOutput("TestElevator/PID output", pidOutput);
    Logger.recordOutput("TestElevator/feedforward Output", feedforwardOutput);

    double input = DriverStation.isEnabled() ? pidOutput + feedforwardOutput : 0;

    sim.setInputVoltage(input);
    sim.update(0.02);

    Logger.recordOutput("TestElevator/volts", input);
    Logger.recordOutput("TestElevator/setpoint", setpoint);
    Logger.recordOutput("TestElevator/Currentheight", getCurrentHeight());
    Logger.recordOutput(
        "TestElevator/CurrentVelocity", Units.metersToInches(sim.getVelocityMetersPerSecond()));
  }

  public double getCurrentHeight() {
    return Units.metersToInches(sim.getPositionMeters());
  }

  public void setTargetHeight(double height) {
    setpoint = height;
  }

  public void setVoltage(double volts1, double volts2) {}

  @AutoLogOutput(key = "Elevator/isAtTarget")
  public boolean isAtTarget() {
    return Math.abs(getCurrentHeight() - setpoint) <= ElevatorConstants.AT_TARGET_GIVE_INCHES;
  }

  public void updateMech2D() {}

  public void setPID(LoggedTunableGains pid) {
    this.pid = pid.createPIDController();
    feedforward = pid.createElevatorFF();
  }
}
