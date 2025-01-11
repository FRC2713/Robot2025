package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants;
import frc.robot.subsystems.constants.ElevatorConstants;

public class ElevatorIOSim implements ElevatorIO {

  private final DCMotor m_motor = DCMotor.getNEO(2);

  private final ProfiledPIDController m_controller =
      ElevatorConstants.PID.createTrapezoidalPIDController();
  private final ElevatorFeedforward m_feedforward = ElevatorConstants.FF.createElevatorFF();

  private final ElevatorSim m_elevatorSim =
      new ElevatorSim(
          m_motor,
          ElevatorConstants.kGearReduction,
          ElevatorConstants.kCarriageMass,
          ElevatorConstants.kDrumRadius,
          ElevatorConstants.kMinHeight,
          ElevatorConstants.kMaxHeight,
          true,
          ElevatorConstants.kInitialHeight);

  public void updateInputs(ElevatorInputs inputs) {
    double pidOutput = m_controller.calculate(m_elevatorSim.getPositionMeters());
    double feedforwardOutput = m_feedforward.calculate(m_controller.getSetpoint().velocity);
    m_elevatorSim.setInputVoltage(pidOutput + feedforwardOutput);
    m_elevatorSim.update(Constants.simulationRate);

    inputs.currentDrawAmpsLeft = m_elevatorSim.getCurrentDrawAmps();
    inputs.currentDrawAmpsRight = m_elevatorSim.getCurrentDrawAmps();

    inputs.heightInchesLeft = Units.metersToInches(m_elevatorSim.getPositionMeters());
    inputs.heightInchesRight = Units.metersToInches(m_elevatorSim.getPositionMeters());

    inputs.outputVoltageLeft = m_elevatorSim.getOutput(0);
    inputs.outputVoltageRight = m_elevatorSim.getOutput(0);

    inputs.tempCelsiusLeft = 0;
    inputs.tempCelsiusRight = 0;

    inputs.velocityInchesPerSecondLeft =
        Units.metersToInches(m_elevatorSim.getVelocityMetersPerSecond());
    inputs.velocityInchesPerSecondRight =
        Units.metersToInches(m_elevatorSim.getVelocityMetersPerSecond());

    inputs.commandedHeight = Units.metersToInches(m_controller.getGoal().position);
  }

  public void setVoltage(double volts1, double volts2) {
    m_elevatorSim.setInputVoltage((volts1 + volts2) / 2);
  }

  public void setTargetHeight(double heightInches) {
    m_controller.setGoal(Units.inchesToMeters(heightInches));
  }
}
