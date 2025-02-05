package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.subsystems.constants.ElevatorConstants;
import frc.robot.util.LoggedTunablePID;

public class ElevatorIOSim implements ElevatorIO {
  private final DCMotor motor = DCMotor.getKrakenX60Foc(2);

  private PIDController pid = ElevatorConstants.PID.createPIDController();
  private ElevatorFeedforward feedforward = ElevatorConstants.PID.createElevatorFF();
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

  @Override
  public void updateInputs(ElevatorInputs inputs) {
    double pidOutput = pid.calculate(Units.metersToInches(sim.getPositionMeters()), setpoint);
    double feedforwardOutput = feedforward.calculate(pid.getSetpoint());

    double input = pidOutput + feedforwardOutput;

    sim.setInputVoltage(input);
    sim.update(0.02);

    inputs.outputVoltageLeft = input;
    inputs.heightInchesLeft = Units.metersToInches(sim.getPositionMeters());
    inputs.velocityInchesPerSecondLeft = Units.metersToInches(sim.getVelocityMetersPerSecond());
    inputs.tempCelsiusLeft = 0.0;
    inputs.currentDrawAmpsLeft = sim.getCurrentDrawAmps();

    inputs.outputVoltageRight = input;
    inputs.heightInchesRight = Units.metersToInches(sim.getPositionMeters());
    inputs.velocityInchesPerSecondRight = Units.metersToInches(sim.getVelocityMetersPerSecond());
    inputs.tempCelsiusRight = 0.0;
    inputs.currentDrawAmpsRight = sim.getCurrentDrawAmps();

    inputs.commandedHeightInches = setpoint;
  }

  @Override
  public void setTargetHeight(double heightInches) {
    setpoint = heightInches;
  }

  @Override
  public void setPID(LoggedTunablePID pid) {
    this.pid = pid.createPIDController();
    feedforward = pid.createElevatorFF();
  }

  public void setVoltage(double volts1, double volts2) {
    sim.setInputVoltage((volts1 + volts2) / 2.0);
  }

  @Override
  public boolean isAtTarget() {
    return Math.abs(Units.metersToInches(sim.getPositionMeters()) - setpoint)
        <= ElevatorConstants.AT_TARGET_GIVE_INCHES;
  }
}
