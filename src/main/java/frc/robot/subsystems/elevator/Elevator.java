package frc.robot.subsystems.elevator;

public class Elevator extends SubsystemBase {

    private ElevatorInputsAutoLogged inputs;
    private ElevatorIO IO;

    public Elevator(ElevatorIO IO) {
        this.inputs = new ElevatorInputsAutoLogged();
        this.IO = IO;
        this.IO.updateInputs(this.inputs);
    }

    @Override
    public void periodic() {
      this.IO.updateInputs(this.inputs);
      Logger.processInputs("Elevator", this.inputs);
    }

    public void setTargetHeight(double height) {
        this.IO.setTargetHeight(height;)
    }


}
