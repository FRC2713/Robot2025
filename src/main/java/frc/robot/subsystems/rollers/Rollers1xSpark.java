package frc.robot.subsystems.rollers;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.constants.RollerConstants;
import frc.robot.util.DebounceBoolean;
import org.littletonrobotics.junction.Logger;

/** For the first implementation, the robot controls Tube and Algae with a single NEO */
public class Rollers1xSpark implements RollersIO {

  private final SparkMax algaeMotor;
  private double targetRPM;

  private DebounceBoolean currentSensing = new DebounceBoolean();

  public Rollers1xSpark() {
    this.algaeMotor = new SparkMax(RollerConstants.kAlgaeCANId, MotorType.kBrushless);
    algaeMotor.configure(
        RollerConstants.createAlgaeConfig(),
        ResetMode.kResetSafeParameters,
        PersistMode.kNoPersistParameters);
  }

  @Override
  public void updateInputs(RollersInputs inputs) {
    inputs.algaeVelocityRPM = algaeMotor.getEncoder().getVelocity();
    inputs.algaeOutputVoltage = algaeMotor.getAppliedOutput();
    inputs.algaeIsOn =
        Math.abs(Units.rotationsPerMinuteToRadiansPerSecond(algaeMotor.getEncoder().getVelocity()))
            > 0.005;
    inputs.algaeCurrentAmps = algaeMotor.getOutputCurrent();
    inputs.commandedAlgaeRPM = this.targetRPM;

    Logger.recordOutput("Rollers/HasCoral", currentSensing.get());
  }

  public void setTubeRPM(double rpm) {
    this.targetRPM = -rpm;
    algaeMotor.set(-rpm / RollerConstants.kAlgaeMaxVelocity);
    // algaeMotor.getClosedLoopController().setReference(-rpm, ControlType.kVelocity);
  }

  public void setAlgaeRPM(double rpm) {
    this.targetRPM = rpm;
    algaeMotor.set(rpm / RollerConstants.kAlgaeMaxVelocity);
    // algaeMotor.getClosedLoopController().setReference(rpm, ControlType.kVelocity);
  }

  public double getTubeRPM() {
    return -algaeMotor.getEncoder().getVelocity();
  }

  public double getAlgaeRPM() {
    return algaeMotor.getEncoder().getVelocity();
  }

  public boolean hasCoral() {
    return currentSensing.set(Math.abs(algaeMotor.getOutputCurrent()) > 30.0);
  }
}
