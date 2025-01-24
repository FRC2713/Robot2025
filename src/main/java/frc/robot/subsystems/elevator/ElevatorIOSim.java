package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.subsystems.constants.ElevatorConstants;

public class ElevatorIOSim implements ElevatorIO {
  private final DCMotor motor = DCMotor.getNEO(2);

  private final ProfiledPIDController pid = ElevatorConstants.PID.createTrapezoidalPIDController();
  private final ElevatorFeedforward feedforward = ElevatorConstants.FF.createElevatorFF();
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
  public double lastHeight = 0.0;

  @Override
  public void updateInputs(ElevatorInputs inputs) {
    double pidOutput = pid.calculate(Units.metersToInches(sim.getPositionMeters()));
    double feedforwardOutput = feedforward.calculate(pid.getSetpoint().velocity);
    sim.setInputVoltage(pidOutput + feedforwardOutput);
    sim.update(0.02);

    inputs.outputVoltageLeft = sim.getOutput(0);
    inputs.heightInchesLeft = Units.metersToInches(sim.getPositionMeters());
    inputs.velocityInchesPerSecondLeft = Units.metersToInches(sim.getVelocityMetersPerSecond());
    inputs.tempCelsiusLeft = 0.0;
    inputs.currentDrawAmpsLeft = sim.getCurrentDrawAmps();

    inputs.outputVoltageRight = sim.getOutput(0);
    inputs.heightInchesRight = Units.metersToInches(sim.getPositionMeters());
    inputs.velocityInchesPerSecondRight = Units.metersToInches(sim.getVelocityMetersPerSecond());
    inputs.tempCelsiusRight = 0.0;
    inputs.currentDrawAmpsRight = sim.getCurrentDrawAmps();
  }

  @Override
  public void setTargetHeight(double heightInches) {
    pid.setGoal(heightInches);
    lastHeight = heightInches;
  }

  public void setVoltage(double volts1, double volts2) {
    sim.setInputVoltage((volts1 + volts2) / 2.0);
  }

  public boolean isAtTarget() {
    return Math.abs(Units.metersToInches(sim.getPositionMeters()) - lastHeight) <= 1;
  }
}
