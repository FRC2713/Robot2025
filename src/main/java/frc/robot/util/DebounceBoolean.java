package frc.robot.util;

import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.Logger;

public class DebounceBoolean {
  private boolean output = false;
  private final double delay = 0.02;

  boolean debouncing = false;
  Timer t = new Timer();

  public DebounceBoolean() {
    t.start();
  }

  public boolean set(boolean b) {
    if (b != output) {
      if (!debouncing) {
        debouncing = true;
        t.restart();
      }

      if (debouncing && t.hasElapsed(delay)) {
        debouncing = false;
        output = b;
      }
    } else {
      output = b;
    }

    Logger.recordOutput("Rollers/HasCoralDebouncing", debouncing);

    return output;
  }

  public boolean get() {
    return output;
  }

  public boolean override(boolean b) {
    this.output = b;
    return output;
  }
}
