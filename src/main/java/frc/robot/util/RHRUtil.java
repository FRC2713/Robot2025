package frc.robot.util;

import edu.wpi.first.math.geometry.Twist2d;
import frc.robot.Constants;
import frc.robot.Constants.Mode;

public class RHRUtil {
  public static double speed(Twist2d twist) {
    return Math.sqrt(Math.pow(twist.dx, 2) + Math.pow(twist.dy, 2));
  }

  public static double modeDependentDouble(double real) {
    return real;
  }

  public static double modeDependentDouble(double real, double sim) {
    return Constants.currentMode == Mode.SIM ? sim : real;
  }
}
