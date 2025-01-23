package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;

public class ElevatorIOSim implements ElevatorIO {
  private final DCMotor motor = DCMotor.getNEO(2);
  private final ProfiledPIDController pid = new ProfiledPIDController(0, 0, 0, null);
  private final Encoder encoder = new Encoder(42, 42);
  private final ElevatorFeedforward feedforward = new ElevatorFeedforward(0, 0.5, 0.1, 0);
  private final ElevatorSim sim =
      new ElevatorSim(
          motor, 5.0, 0.3, Units.inchesToMeters(1), 0, Units.inchesToMeters(17), true, 0);
  public double lastHeight = 0.0;

  public void setVoltage(double volts1, double volts2) {
    sim.setInputVoltage((volts1 + volts2) / 2.0);
  }

  @Override
  public void updateInputs(ElevatorInputs inputs) {
    double pidOutput = pid.calculate(encoder.getDistance());
    double feedforwardOutput = feedforward.calculate(pid.getSetpoint().velocity);
    sim.setInputVoltage(pidOutput + feedforwardOutput);

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

    sim.update(0.02);
  }

  @Override
  public void reset() {
    throw new UnsupportedOperationException("Unimplemented method 'reset'");
  }

  @Override
  public boolean shouldApplyFF() {
    throw new UnsupportedOperationException("Unimplemented method 'shouldApplyFF'");
  }

  @Override
  public void setTargetHeight(double heightInches) {
    pid.setGoal(heightInches);
    lastHeight = heightInches;
  }

  @Override
  public void setCurrentLimits() {
    throw new UnsupportedOperationException("Unimplemented method 'setCurrentLimits'");
  }

  public boolean isAtTarget() {
    return Math.abs(Units.metersToInches(sim.getPositionMeters()) - lastHeight) <= 1;
  }
}
