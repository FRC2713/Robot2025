package frc.robot.subsystems.rollers;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.constants.RollerConstants;

public class RollersSparks implements RollersIO {

  private final SparkMax tubeMotor;
  private final SparkMax algaeMotor;
  private double commandedAlgaeRPM, commandedTubeRPM;

  public RollersSparks() {
    this.tubeMotor = new SparkMax(101, MotorType.kBrushless);
    this.tubeMotor.configure(
        RollerConstants.createCoralConfig(),
        ResetMode.kResetSafeParameters,
        PersistMode.kNoPersistParameters);
    this.algaeMotor = new SparkMax(101, MotorType.kBrushless);
    this.algaeMotor.configure(
        RollerConstants.createAlgaeConfig(),
        ResetMode.kResetSafeParameters,
        PersistMode.kNoPersistParameters);
  }

  public void updateInputs(RollersInputs inputs) {
    inputs.algaeVelocityRPM = algaeMotor.getEncoder().getVelocity();
    inputs.algaeOutputVoltage = algaeMotor.getAppliedOutput();
    inputs.algaeIsOn =
        Math.abs(Units.rotationsPerMinuteToRadiansPerSecond(algaeMotor.getEncoder().getVelocity()))
            > 0.005;
    inputs.algaeCurrentAmps = algaeMotor.getOutputCurrent();
    inputs.commandedAlgaeRPM = this.commandedAlgaeRPM;

    inputs.tubeVelocityRPM = tubeMotor.getEncoder().getVelocity();
    inputs.tubeOutputVoltage = tubeMotor.getAppliedOutput();
    inputs.tubeIsOn =
        Math.abs(Units.rotationsPerMinuteToRadiansPerSecond(tubeMotor.getEncoder().getVelocity()))
            > 0.005;
    inputs.tubeCurrentAmps = tubeMotor.getOutputCurrent();
    inputs.commandedTubeRPM = this.commandedTubeRPM;
  }

  public void setTubeRPM(double rpm) {
    tubeMotor.getClosedLoopController().setReference(rpm, ControlType.kMAXMotionVelocityControl);
  }

  public void setAlgaeRPM(double rpm) {
    algaeMotor.getClosedLoopController().setReference(rpm, ControlType.kMAXMotionVelocityControl);
  }
  ;
}
