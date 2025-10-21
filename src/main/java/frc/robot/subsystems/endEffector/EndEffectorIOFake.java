package frc.robot.subsystems.endEffector;

public class EndEffectorIOFake implements EndEffectorIO {

  public void updateInputs(EndEffectorInputs inputs) {}
  ;

  public void setAlgaeRPM(double rpm) {}
  ;

  public void setAlgaeVoltage(double volts) {}
  ;

  public void setCoralRPM(double rpm) {}
  ;

  public void setEnableLimitSwitch(boolean enabled) {}
  ;

  public boolean isCoralAtTarget() {
    return true;
  }

  public boolean isAlgaeAtTarget() {
    return true;
  }

  public void setCoralCurrentLimit(int currentLimit) {}

  public void setAlgaeCurrentLimit(int algaeCurrentLimit) {}
}
