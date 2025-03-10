package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import lombok.Getter;

public class TrapezoidZone {

  private final Point topLeft;
  private final Point topRight;
  private final Point bottomLeft;
  private final Point bottomRight;
  @Getter private final Rotation2d omega;

  public TrapezoidZone(
      Point topLeft, Point topRight, Point bottomLeft, Point bottomRight, Rotation2d omega) {
    this.topLeft = topLeft;
    this.topRight = topRight;
    this.bottomLeft = bottomLeft;
    this.bottomRight = bottomRight;
    this.omega = omega;
  }

  public TrapezoidZone(
      Point baseLeft, Point baseRight, double height, Rotation2d angle, boolean invert, Rotation2d omega) {
    topLeft = baseLeft;
    topRight = baseRight;
    var top_edge_angle = Math.atan2(topRight.y - topLeft.y, topRight.x - topLeft.x);
    var angle_radians = angle.getRadians();
    var left_side_angle = top_edge_angle - Math.PI / 2 + angle_radians;
    var right_side_angle = top_edge_angle + Math.PI / 2 - angle_radians;


    var left_side_length = height / Math.sin(angle_radians);
    var right_side_length = height / Math.sin(angle_radians);
    bottomLeft =
    invert ? 
    new Point(
      topLeft.x -(left_side_length * Math.cos(right_side_angle)),
      topLeft.y - (left_side_length * Math.sin(right_side_angle)))
        : 
         new Point(
              topLeft.x - (left_side_length * Math.cos(left_side_angle)),
              topLeft.y - (left_side_length * Math.sin(left_side_angle)));
    bottomRight =
    invert ? new Point(
      topRight.x + (right_side_length * Math.cos(left_side_angle)),
      topRight.y + (right_side_length * Math.sin(left_side_angle))) :

        new Point(
            topRight.x+ (right_side_length * Math.cos(right_side_angle)),
            topRight.y +(right_side_length * Math.sin(right_side_angle)));
    this.omega = omega;
  }

  public boolean isPointInside(Pose2d p) {
    // Check if the point is within the bounds of the trapezoid's sides.
    // We can do this by checking if the point is on the correct side of each line segment.

    var pose = AllianceFlipUtil.apply(p);
    return isPointOnCorrectSide(pose, topLeft, topRight, bottomLeft)
        && isPointOnCorrectSide(pose, topRight, bottomRight, bottomLeft)
        && isPointOnCorrectSide(pose, bottomRight, bottomLeft, topLeft)
        && isPointOnCorrectSide(pose, bottomLeft, topLeft, topRight);
  }

  private boolean isPointOnCorrectSide(Pose2d p, Point lineStart, Point lineEnd, Point reference) {
    // Calculate the cross product of (lineEnd - lineStart) and (p - lineStart).
    // The sign of the cross product indicates which side of the line the point is on.
    // We compare this sign to the sign of the cross product of (lineEnd - lineStart) and (reference
    // - lineStart).
    // If the signs are the same, the point is on the correct side.

    double crossProductP =
        crossProduct(
            lineStart, lineEnd, new Point(p.getTranslation().getX(), p.getTranslation().getY()));
    double crossProductRef = crossProduct(lineStart, lineEnd, reference);

    return Math.signum(crossProductP) == Math.signum(crossProductRef);
  }

  private double crossProduct(Point a, Point b, Point c) {
    return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
  }

  public static class Point {
    public double x;
    public double y;

    public Point(double x, double y) {
      this.x = x;
      this.y = y;
    }

    @Override
    public String toString() {
      return "(" + x + ", " + y + ")";
    }

    public Pose2d toPose2d() {
      return AllianceFlipUtil.apply(new Pose2d(new Translation2d(x, y), new Rotation2d()));
    }

    public double distanceTo(Point other) {
      return Math.hypot(other.x - x, other.y - y);
    }
  }

  public Pose2d[] toPose2dArray() {
    return new Pose2d[] {
      topLeft.toPose2d(),
      topRight.toPose2d(),
      bottomRight.toPose2d(),
      bottomLeft.toPose2d(),
      topLeft.toPose2d()
    };
  }
}
