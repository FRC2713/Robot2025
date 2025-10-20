package frc.robot.subsystems.endEffector;

/** For the first implementation, the robot controls Tube and Algae with a single NEO */
public class EndEffectorIOSparks implements EndEffectorIO {

  public EndEffectorIOSparks() {}

  public void updateInputs(EndEffectorInputs inputs) {}

  @Override
  public void setCoralRPM(double rpm) {}

  @Override
  public void setAlgaeRPM(double rpm) {}

  @Override
  public boolean isCoralAtTarget() {
    return false;
  }

  @Override
  public boolean isAlgaeAtTarget() {
    return false;
  }

  @Override
  public void setEnableLimitSwitch(boolean setEnable) {}

  @Override
  public void setCoralCurrentLimit(int currentLimit) {}

  @Override
  public void setAlgaeCurrentLimit(int algaeCurrentLimit) {}
}
