package frc.robot.util;

import edu.wpi.first.math.geometry.Twist2d;

public class RHRUtil {
  public static double speed(Twist2d twist) {
    return Math.sqrt(Math.pow(twist.dx, 2) + Math.pow(twist.dy, 2));
  }
}
