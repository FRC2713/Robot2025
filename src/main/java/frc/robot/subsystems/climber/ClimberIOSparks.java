package frc.robot.subsystems.climber;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SoftLimitConfig;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Servo;
import frc.robot.subsystems.constants.ClimberConstants;
import frc.robot.util.LoggedTunableGains;

public class ClimberIOSparks implements ClimberIO {
  public SparkFlex leftMotor = new SparkFlex(ClimberConstants.kLeftCANId, MotorType.kBrushless);
  public SparkFlex rightMotor = new SparkFlex(ClimberConstants.kRightCANId, MotorType.kBrushless);
  public Servo servo = new Servo(0);
  private double target;

  public ClimberIOSparks() {
    leftMotor.configure(
        ClimberConstants.createLeftSparkConfig(ClimberConstants.initialSoftLimits),
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    leftMotor.getEncoder().setPosition(Units.degreesToRotations(ClimberConstants.kInitialAngle));
    rightMotor.configure(
        ClimberConstants.createRightSparkConfig(ClimberConstants.initialSoftLimits),
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    rightMotor.getEncoder().setPosition(Units.degreesToRotations(ClimberConstants.kInitialAngle));
  }

  @Override
  public void updateInputs(ClimberInputs inputs) {
    inputs.leftVelocityDPS = Units.rotationsToDegrees(leftMotor.getEncoder().getVelocity());
    inputs.leftVoltage = leftMotor.getAppliedOutput() * leftMotor.getBusVoltage();
    inputs.leftAmps = leftMotor.getOutputCurrent();
    inputs.leftAngleDegrees = Units.rotationsToDegrees(leftMotor.getEncoder().getPosition());

    inputs.rightVelocityDPS = Units.rotationsToDegrees(rightMotor.getEncoder().getVelocity());
    inputs.rightVoltage = rightMotor.getAppliedOutput() * rightMotor.getBusVoltage();
    inputs.rightAmps = rightMotor.getOutputCurrent();
    inputs.rightAngleDegrees = Units.rotationsToDegrees(rightMotor.getEncoder().getPosition());

    inputs.servoCommandedPos = servo.getPosition();

    inputs.commandedAngleDegs = this.target;
  }

  @Override
  public void setPID(LoggedTunableGains pid) {
    leftMotor.configureAsync(
        ClimberConstants.createLeftSparkConfig(ClimberConstants.initialSoftLimits),
        ResetMode.kNoResetSafeParameters,
        PersistMode.kNoPersistParameters);
    rightMotor.configureAsync(
        ClimberConstants.createRightSparkConfig(ClimberConstants.initialSoftLimits),
        ResetMode.kNoResetSafeParameters,
        PersistMode.kNoPersistParameters);
  }

  @Override
  public void setServoPos(double pos) {
    servo.set(pos);
  }

  @Override
  public void setTargetAngle(double angle) {
    this.target = angle;
    leftMotor
        .getClosedLoopController()
        .setReference(
            Units.degreesToRotations(angle), ControlType.kPosition, ClosedLoopSlot.kSlot0);
    rightMotor
        .getClosedLoopController()
        .setReference(
            Units.degreesToRotations(angle), ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  @Override
  public void setVoltage(double volts) {
    leftMotor.setVoltage(volts);
    // rightMotor.setVoltage(volts);
  }

  @Override
  public void configureSoftLimits(double minDeg, double maxDeg) {
    var limits =
        new SoftLimitConfig()
            .forwardSoftLimitEnabled(true)
            .forwardSoftLimit(Units.degreesToRotations(maxDeg))
            .reverseSoftLimitEnabled(true)
            .reverseSoftLimit(Units.degreesToRotations(minDeg));
    leftMotor.configureAsync(
        ClimberConstants.createLeftSparkConfig(limits),
        ResetMode.kResetSafeParameters,
        PersistMode.kNoPersistParameters);
    rightMotor.configureAsync(
        ClimberConstants.createRightSparkConfig(limits),
        ResetMode.kResetSafeParameters,
        PersistMode.kNoPersistParameters);
  }
}
